/**
 * @file    SCS.c
 * @brief   SCS 串行舵机通信协议层
 *
 * 负责协议帧的封装(打包)与解析(解包)、校验和计算，以及
 * 单舵机/多舵机(同步)的读写指令与应答处理。
 *
 * 层次关系（自底向上）:
 *   uart.c      底层 UART 收发(HAL 驱动)
 *   SCSerail.c  硬件接口层(发送缓冲/接收超时/总线切换)
 *   SCS.c       协议层(本文件: 帧格式、校验、读写指令)  <-- 本文件
 *   SMS_STS.c   应用层(舵机内存表高层封装: 写位置/读反馈等)
 *
 * 说明:
 *   - 应用层一般直接调用 SMS_STS.h 中的函数(如 WritePosEx、ReadPos)，
 *     本文件中的函数通常由 SMS_STS.c 内部调用。
 *   - SCS 通信帧格式: [0xFF][0xFF][ID][LEN][INST][...参数...][SUM]
 *     校验和 SUM = ~(ID + LEN + INST + 参数1 + ... + 参数n)  (按字节累加后取反)
 *   - 0xFE 为广播 ID，表示对总线上所有舵机操作。
 *
 * 日期: 2022.3.30
 */

#include <stdlib.h>
#include "INST.h"
#include "SCS.h"

static uint8_t Level = 1;   // 舵机应答等级: 1=发送指令后等待并校验应答, 0=不等待应答(用于广播等)
static uint8_t End = 0;     // 大小端标志: 0=小端(低字节在前), 1=大端(高字节在前)
static uint8_t Error = 0;   // 最近一次通信读到的舵机状态/错误码

/* 同步读(SYNC READ)相关接收缓冲区 */
uint8_t syncReadRxPacketIndex;   // 当前解码指针位置
uint8_t syncReadRxPacketLen;    // 每帧数据的长度
uint8_t *syncReadRxPacket;      // 指向当前正在解码的返回数据
uint8_t *syncReadRxBuff;        // 同步读的原始接收缓冲区
uint16_t syncReadRxBuffLen;     // 同步读实际接收到的字节数
uint16_t syncReadRxBuffMax;     // 同步读接收缓冲区的最大容量

/**
 * @brief  获取最近一次通信的舵机状态/错误码
 * @retval 舵机状态字节(含义参见飞特舵机协议的错误码表)
 */
uint8_t getSCSErr(void)
{
	return Error;
}

/**
 * @brief  将一个 16 位数拆分为两个 8 位数(按当前大小端顺序)
 * @param  DataL 输出: 低字节
 * @param  DataH 输出: 高字节
 * @param  Data  输入的 16 位数值
 * @note   具体哪个在前由全局变量 End(大小端)决定
 */
void Host2SCS(uint8_t *DataL, uint8_t* DataH, int Data)
{
	if(End){
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	}else{
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}

/**
 * @brief  将两个 8 位数组合为一个 16 位数(按当前大小端顺序)
 * @param  DataL 低字节
 * @param  DataH 高字节
 * @retval 组合后的 16 位数值
 */
int SCS2Host(uint8_t DataL, uint8_t DataH)
{
	int Data;
	if(End){
		Data = DataL;
		Data<<=8;
		Data |= DataH;
	}else{
		Data = DataH;
		Data<<=8;
		Data |= DataL;
	}
	return Data;
}

/**
 * @brief  协议层基础打包函数：把一条指令封装成帧并写入发送缓冲区
 * @param  ID      舵机 ID(0xFE 为广播)
 * @param  MemAddr 舵机内存表地址
 * @param  nDat    参数数据指针(可为 NULL，表示无参数)
 * @param  nLen    参数长度
 * @param  Fun     指令码(见 INST.h: INST_WRITE/INST_READ/INST_PING 等)
 * @note   帧格式: [0xFF][0xFF][ID][LEN][INST][MemAddr][参数...][SUM]
 */
void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
{
	uint8_t i;
	uint8_t msgLen = 2;
	uint8_t bBuf[6];
	uint8_t CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;
	if(nDat){
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		writeSCS(bBuf, 6);
		
	}else{
		bBuf[3] = msgLen;
		writeSCS(bBuf, 5);
	}
	CheckSum = ID + msgLen + Fun + MemAddr;
	if(nDat){
		for(i=0; i<nLen; i++){
			CheckSum += nDat[i];
		}
		writeSCS(nDat, nLen);
	}
	CheckSum = ~CheckSum;
	writeSCS(&CheckSum, 1);
}

/**
 * @brief  普通写指令(立即生效)
 * @param  ID      舵机 ID
 * @param  MemAddr 内存表地址
 * @param  nDat    要写入的数据
 * @param  nLen    数据长度(字节)
 * @retval 1 成功 / 0 失败(应答超时或校验错)
 */
int genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

/**
 * @brief  异步写指令(先缓存，不立即执行)
 * @param  ID      舵机 ID
 * @param  MemAddr 内存表地址
 * @param  nDat    要写入的数据
 * @param  nLen    数据长度(字节)
 * @retval 1 成功 / 0 失败
 * @note   需配合 regAction() 统一触发执行
 */
int regWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
	wFlushSCS();
	return Ack(ID);
}

/**
 * @brief  异步写执行指令：让之前 regWrite 缓存的数据生效
 * @param  ID 舵机 ID(通常传 0xFE 广播，让所有缓存同时生效)
 * @retval 1 成功 / 0 失败
 */
int regAction(uint8_t ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_REG_ACTION);
	wFlushSCS();
	return Ack(ID);
}

/**
 * @brief  同步写指令：一帧内同时写多个舵机(广播 0xFE)
 * @param  ID      舵机 ID 数组
 * @param  IDN     ID 数量
 * @param  MemAddr 内存表地址
 * @param  nDat    数据缓冲区(按 ID 顺序，每个 ID 占 nLen 字节)
 * @param  nLen    每个舵机的数据长度
 * @note   多舵机需动作同步时用此函数，比逐个写更快、更同步
 */
void syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	uint8_t mesLen = ((nLen+1)*IDN+4);
	uint8_t Sum = 0;
	uint8_t bBuf[7];
	uint8_t i, j;
	
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	
	rFlushSCS();
	writeSCS(bBuf, 7);

	Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;

	for(i=0; i<IDN; i++){
		writeSCS(&ID[i], 1);
		writeSCS(nDat+i*nLen, nLen);
		Sum += ID[i];
		for(j=0; j<nLen; j++){
			Sum += nDat[i*nLen+j];
		}
	}
	Sum = ~Sum;
	writeSCS(&Sum, 1);
	wFlushSCS();
}

/**
 * @brief  写 1 个字节到指定内存表地址
 * @retval 1 成功 / 0 失败
 */
int writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

/**
 * @brief  写 2 个字节(16 位)到指定内存表地址
 * @retval 1 成功 / 0 失败
 */
int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat)
{
	uint8_t buf[2];
	Host2SCS(buf+0, buf+1, wDat);
	rFlushSCS();
	writeBuf(ID, MemAddr, buf, 2, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

/**
 * @brief  读指令：从舵机内存表读取 nLen 字节数据
 * @param  ID      舵机 ID
 * @param  MemAddr 内存表起始地址
 * @param  nData   输出: 读到的数据缓冲区
 * @param  nLen    要读取的字节数
 * @retval 实际读到的字节数; 0 表示失败(超时/校验错)
 * @note   通信状态可通过 getSCSErr() 获取
 */
int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen)
{
	int Size;
	uint8_t bBuf[4];
	uint8_t calSum;
	uint8_t i;
	rFlushSCS();
	writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
	wFlushSCS();
	if(!checkHead()){
		return 0;
	}
	Error = 0;
	if(readSCS(bBuf, 3)!=3){
		return 0;
	}
	Size = readSCS(nData, nLen);
	if(Size!=nLen){
		return 0;
	}
	if(readSCS(bBuf+3, 1)!=1){
		return 0;
	}
	calSum = bBuf[0]+bBuf[1]+bBuf[2];
	for(i=0; i<Size; i++){
		calSum += nData[i];
	}
	calSum = ~calSum;
	if(calSum!=bBuf[3]){
		return 0;
	}
	Error = bBuf[2];
	return Size;
}

/**
 * @brief  读 1 字节
 * @retval 读到数值; -1 表示失败
 */
int readByte(uint8_t ID, uint8_t MemAddr)
{
	uint8_t bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if(Size!=1){
		return -1;
	}else{
		return bDat;
	}
}

/**
 * @brief  读 2 字节(16 位)
 * @retval 读到数值; -1 表示失败
 */
int readWord(uint8_t ID, uint8_t MemAddr)
{	
	uint8_t nDat[2];
	int Size;
	uint16_t wDat;
	Size = Read(ID, MemAddr, nDat, 2);
	if(Size!=2)
		return -1;
	wDat = SCS2Host(nDat[0], nDat[1]);
	return wDat;
}

/**
 * @brief  Ping 指令：查询舵机是否存在
 * @param  ID 舵机 ID(0xFE 表示广播)
 * @retval 返回舵机 ID; -1 表示超时/无应答
 */
int	Ping(uint8_t ID)
{
	uint8_t bBuf[4];
	uint8_t calSum;
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_PING);
	wFlushSCS();
	Error = 0;
	if(!checkHead()){
		return -1;
	}
	
	if(readSCS(bBuf, 4)!=4){
		return -1;
	}
	if(bBuf[0]!=ID && ID!=0xfe){
		return -1;
	}
	if(bBuf[1]!=2){
		return -1;
	}
	calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if(calSum!=bBuf[3]){
		return -1;			
	}
	Error = bBuf[2];
	return bBuf[0];
}

/**
 * @brief  检测并跳过数据流中的帧头(连续两个 0xFF)
 * @retval 1 找到帧头 / 0 超时或未找到
 */
int checkHead(void)
{
	uint8_t bDat;
	uint8_t bBuf[2] = {0, 0};
	uint8_t Cnt = 0;
	while(1){
		if(!readSCS(&bDat, 1)){
			return 0;
		}
		bBuf[1] = bBuf[0];
		bBuf[0] = bDat;
		if(bBuf[0]==0xff && bBuf[1]==0xff){
			break;
		}
		Cnt++;
		if(Cnt>10){
			return 0;
		}
	}
	return 1;
}

/**
 * @brief  等待并校验舵机应答帧
 * @param  ID 舵机 ID
 * @retval 1 应答正常 / 0 超时或校验错
 * @note   只有当 Level=1 且 ID!=0xFE 时才真正等待应答
 */
int	Ack(uint8_t ID)
{
	uint8_t bBuf[4];
	uint8_t calSum;
	Error = 0;
	if(ID!=0xfe && Level){
		if(!checkHead()){
			return 0;
		}
		if(readSCS(bBuf, 4)!=4){
			return 0;
		}
		if(bBuf[0]!=ID){
			return 0;
		}
		if(bBuf[1]!=2){
			return 0;
		}
		calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
		if(calSum!=bBuf[3]){
			return 0;			
		}
		Error = bBuf[2];
	}
	return 1;
}

/**
 * @brief  同步读指令包发送(广播 0xFE)，并接收返回数据到缓冲区
 * @param  ID      舵机 ID 数组
 * @param  IDN     ID 数量
 * @param  MemAddr 内存表起始地址
 * @param  nLen    每个舵机要读的字节数
 * @retval 实际接收到的字节数
 * @note   使用前需先 syncReadBegin() 申请缓冲区，用后 syncReadEnd() 释放
 */
int	syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen)
{
	uint8_t checkSum;
	uint8_t i;
	rFlushSCS();
	syncReadRxPacketLen = nLen;
	checkSum = (4+0xfe)+IDN+MemAddr+nLen+INST_SYNC_READ;
	writeByteSCS(0xff);
	writeByteSCS(0xff);
	writeByteSCS(0xfe);
	writeByteSCS(IDN+4);
	writeByteSCS(INST_SYNC_READ);
	writeByteSCS(MemAddr);
	writeByteSCS(nLen);
	for(i=0; i<IDN; i++){
		writeByteSCS(ID[i]);
		checkSum += ID[i];
	}
	checkSum = ~checkSum;
	writeByteSCS(checkSum);
	wFlushSCS();
	
	syncReadRxBuffLen = readSCS(syncReadRxBuff, syncReadRxBuffMax);
	return syncReadRxBuffLen;
}

/**
 * @brief  同步读开始：按舵机数量与数据长度申请接收缓冲区
 * @param  IDN   舵机数量
 * @param  rxLen 每个舵机的数据长度
 */
void syncReadBegin(uint8_t IDN, uint8_t rxLen)
{
	syncReadRxBuffMax = IDN*(rxLen+6);
	syncReadRxBuff = malloc(syncReadRxBuffMax);
}

/**
 * @brief  同步读结束：释放接收缓冲区
 */
void syncReadEnd(void)
{
	if(syncReadRxBuff){
		free(syncReadRxBuff);
		syncReadRxBuff = NULL;
	}
}

/**
 * @brief  同步读返回包解码：从接收缓冲区中提取指定 ID 的一帧数据
 * @param  ID   要提取的舵机 ID
 * @param  nDat 输出: 解码后的数据
 * @retval 成功返回数据字节数; 0 表示未找到或校验错
 * @note   解码后可用 syncReadRxPacketToByte()/syncReadRxPacketToWrod() 逐项取值
 */
int syncReadPacketRx(uint8_t ID, uint8_t *nDat)
{
	uint16_t syncReadRxBuffIndex = 0;
	syncReadRxPacket = nDat;
	syncReadRxPacketIndex = 0;
	while((syncReadRxBuffIndex+6+syncReadRxPacketLen)<=syncReadRxBuffLen){
		uint8_t bBuf[] = {0, 0, 0};
		uint8_t calSum = 0;
		while(syncReadRxBuffIndex<syncReadRxBuffLen){
			bBuf[0] = bBuf[1];
			bBuf[1] = bBuf[2];
			bBuf[2] = syncReadRxBuff[syncReadRxBuffIndex++];
			if(bBuf[0]==0xff && bBuf[1]==0xff && bBuf[2]!=0xff){
				break;
			}
		}
		if(bBuf[2]!=ID){
			continue;
		}
		if(syncReadRxBuff[syncReadRxBuffIndex++]!=(syncReadRxPacketLen+2)){
			continue;
		}
		Error = syncReadRxBuff[syncReadRxBuffIndex++];
		calSum = ID+(syncReadRxPacketLen+2)+Error;
		for(uint8_t i=0; i<syncReadRxPacketLen; i++){
			syncReadRxPacket[i] = syncReadRxBuff[syncReadRxBuffIndex++];
			calSum += syncReadRxPacket[i];
		}
		calSum = ~calSum;
		if(calSum!=syncReadRxBuff[syncReadRxBuffIndex++]){
			return 0;
		}
		return syncReadRxPacketLen;
	}
	return 0;
}

/**
 * @brief  从解码出的数据包中顺序取出 1 个字节
 * @retval 数值; -1 表示已取完
 */
int syncReadRxPacketToByte(void)
{
	if(syncReadRxPacketIndex>=syncReadRxPacketLen){
		return -1;
	}
	return syncReadRxPacket[syncReadRxPacketIndex++];
}

/**
 * @brief  从解码出的数据包中顺序取出 2 个字节(16 位)
 * @param  negBit 符号位位置(如 15 表示第 15 位为符号位); 0 表示无符号
 * @retval 数值; -1 表示已取完
 */
int syncReadRxPacketToWrod(uint8_t negBit)
{
	if((syncReadRxPacketIndex+1)>=syncReadRxPacketLen){
		return -1;
	}
	int Word = SCS2Host(syncReadRxPacket[syncReadRxPacketIndex], syncReadRxPacket[syncReadRxPacketIndex+1]);
	syncReadRxPacketIndex += 2;
	if(negBit){
		if(Word&(1<<negBit)){
			Word = -(Word & ~(1<<negBit));
		}
	}
	return Word;
}
