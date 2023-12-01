/*
 * inputter_rs485.c
 *
 * About the RS485 bus:
 *
 * Host control the bus and send request commands. Devices listen and operate, answer the response when needed,
 * in which state, host will release and give out the bus and read the response, then retake the bus (no need to do anything, bus belongs to host).
 *
 * The gap between last STOP bit from host request, and the START bit from device response, is at least 2-bit time.
 * As an implementation suggestion, it is best for device to reply after detecting an IDLE frame from bus.
 * The gap is also suitable for the host, when it takes back the bus and before it sends a new request
 *
 * If device's response is timeout, host will send some request to claim the bus, and device should give up trying to continue answering.
 *
 * Devices, if they want to answer some requests, just send data to bus following protocol rules, and no extra data can be sent.
 * So if some bus conflicts happens, devices can not save or even inform the host.
 *
 * Host, should listen the the bus when it is sending data, and compare the sent data and received data,
 * maybe there's still chance to at least give up on completing the entire request.
 *
 * About the request, response and motion control stream:
 *
 * There are 3 types of messages in bus, request, response and motion control stream. All of them are start with an address byte 0x80-0xFF,
 * 0x80-0xFF is address, and 0x00-0x7F is data.
 *
 * If a byte with MSB 1 appears, a new transmission is start, and the last request/response transmission is dropped if it's not completed,
 * and the last motion control stream is just masked, switched, not even stopped.
 *
 * About device and its ID:
 *
 * There's only one host, multiple devices. Device has an ID, if the MSB of ID is set, the device address is got.
 * Device's ID can be set by user, written in FLASH, if ID is requied to be changed, it's better to save the new value to FLASH.
 * Or host must remember which device's ID cannot be modified and will not be reassigned.
 *
 * Because the addresses down from the highest number 0xFF are occupied by the management and control channels, ID 0 is reserved too,
 * Device should not use those ID. So devices should use the number indexed from 1, as their ID.
 * Although the range of device ID is compressed, normally host/devices are not designed to provide such a large current to drive a bus with so many devices.
 *
 * If device's ID is 0, in facts, that means it's not set, it can't go live, it must play deaf and dumb, event when the request is sent to address 0x80,
 * Remember, address 0x80 and device ID 0 is not really for a device who has an ID 0. ID 0 is special and reserved
 *
 * About Time:
 *
 * Device has its time system, host should know this or can be configured to know this
 *
 * About device initialized state:
 *
 * Normal devices can becomes enabled state when they are powered or reset.
 * But motion devices, should be in disabled state when they are powered or reset.
 *
 * About default parameters
 *
 * All parameter has their default value if they are not reconfigured.
 *
 * default baudrate of RS485 bus : 250kbps
 * default response soft timeout : 5s  (means device can send a delay signal to add more waiting time to itself)
 * default response hard timeout : 30s (no host can bear to wait so long without getting the final result)
 *
 * Protocol BNF (non-standard):
 *
 * ; basic data types
 * UINT7  = %x00-7F ; a 7bit unsigned integer, concatenation of 2 low-7-bits, big-endian
 * UINT14 = %x00-7F %x00-7F ; a 14bit unsigned integer, concatenation of 2 low-7-bits, big-endian
 * UINT28 = %x00-7F %x00-7F %x00-7F %x00-7F ; a 28bit unsigned integer, concatenation of 4 low-7-bits, big-endian
 *
 * INT7  = %x00-7F ; a 7bit signed integer, concatenation of 2 low-7-bits, big-endian
 * INT14 = %x00-7F %x00-7F ; a 14bit signed integer, concatenation of 2 low-7-bits, big-endian
 * INT28 = %x00-7F %x00-7F %x00-7F %x00-7F ; a 28bit signed integer, concatenation of 4 low-7-bits, big-endian
 *
 * Text = TextLength TextContent ; 7bit text string
 * TextLength  = UINT14      ; it's length of following TextContent bytes
 * TextContent = *%x00-7F
 *
 * BLob = BlobLength BlobContent ; binary data
 * BlobLength  = UINT14      ; it's length of following BlobContent bytes
 * BlobContent = *%x00-7F    ; concatenation of all 8bit, and using 8 7-bit bytes to express 7 8-bit bytes
 *
 * Parameter = UINT7 / UINT14 / UINT28 / INT7 / INT14 / INT28 / Text / Bolb
 * Parameters = *Parameter
 *
 * DeviceID  = %x01-%x7E
 * DeviceAll = %x00     ; an ID for all the working devices to which the host can send a request
 *
 * DeviceState = DeviceStateDisable / DeviceStateEnable / DeviceStateCustom
 * DeviceStateDisable = %x00 ; device is disabled, ignore all communition other than Manage
 * DeviceStateEnable = %x01 ; device is enabled
 * DeviceStateCustom = %x10-7F ; customized state
 *
 * ; for this device
 * DeviceStateMotion0 = %x10 ; device is enabled, and device plays a role of motion device, using algorithm 0 (basic, open-loop)
 * DeviceStateMotion1 = %x11 ; device is enabled, and device plays a role of motion device, using algorithm 1 (pid)
 * DeviceStateMotion2 = %x12 ; device is enabled, and device plays a role of motion device, using algorithm 2
 * ...
 *
 * Address = %x80-FF
 * Content = *%x00-7F
 * RS485 = *(Address *Content)
 *
 * ; in function level
 * AllFunction = *( Private / Customized / Public / Reserved )
 *
 * Private = PrivateAddress Content ; it's not declared in this protocol
 * PrivateAddress = %x80-DF         ; it's for traditional RS485 communication
 *
 * Customized = CustomizedAddress Content  ; it's hold for other motion protocol owner to send their stream over this protocol, like Klipper MCU command
 * CustomizedAddress = %d249-252
 *
 * Reserved = ReservedAddress Content ; they are reserved for usage in future
 * ReservedAddress = %x80 / %d224-%255 ; except the address occupied by Public
 *
 * Public = Emergency / Manage / StepStream / AdvancedStepStream / MoveCmd / Sampling
 *
 * ; if Emergency signal presents in the bus, all the devices, including those disabled/suspended devices,
 * ; must do the right thing immediately to stop whole system avoiding danger and damage.
 * ; this can be sent by host or devices at any time, in any frequency, but a good time, for example the bus is idle,
 * ; and a good frequency, for example 1 time every several second (in this case, there is almost no device damage due to bus conflicts), is recommended.
 * ; one possible scenario is when the host finds that the system and bus is completely out of control, it may repeate sending this in a period.
 * ; Text is optional, can be a message to indicate the reason.
 * ; if a message using other address other %d126 is used, system exits emergency state
 * Emergency = %d254 [Text]
 *
 * ; host send a manage request to a device with a ID, or to all device in working state
 * ; device should answer the repsonse, unless the request is send to all devices (DeviceAll
 * ; but if the request is send to all devices (DeviceAll: ID=0), devices just operate, don't answer.
 * Manage = ManageToDevice / ManageToAllDevice
 * ManageToDevice =  %d253 DeviceID MangeReq MangeResp
 * ManageToAllDevice = %d253 DeviceAll MangeReq
 *
 * ManageReq = MangeReqCode MangeReqLength MangeReqContent
 * MangeReqLength = UIN14 ; the length of following MangeReqContent
 * MangeReqContent = Parameters
 *
 * MangeResp = 1*(MangeRespCode MangeRespLength MangeRespContent)
 * MangeRespCode = MangeRespCodeWait / MangeRespCodeMore / MangeRespCodeOK / MangeRespCodeFail / MangeRespCodeNoSup / MangeRespCodeBadPro / MangeRespCodeBadOp
 * MangeRespLength = UINT14 ; the length of following MangeRespContent
 * MangeRespContent = Parameters
 *
 * MangeRespCodeWait = %d2   ; tell the host to wait, reset host's soft timeout timer
 * MangeRespCodeMore = %d1   ; operation OK, MangeReqContent contain response, and there are more MangeResp following, until the last MangeRespCodeOK
 * MangeRespCodeOK = %d0     ; operation OK, MangeReqContent contain response
 * MangeRespCodeFail = %d3   ; tried, and failed
 * MangeRespCodeNoSup = %d4  ; MangeReqCode is not supported
 * MangeRespCodeBadPro = %d5 ; the protocol of MangeReqLength and MangeReqContent seems to be bad
 * MangeRespCodeBadOp = %d6  ; the parameters in MangeReqContent if out of device's ability
 *
 * MangeReqCode = ManageReqCodePing / ManageReqCodeBus /
 *                ManageReqCodeReset / ManageReqCodeState / ManageReqCodeDeviceStatus /
 *                ManageReqCodeDeviceStatus / ManageReqCodeDeviceGet / ManageReqCodeDeviceSet /
 *                ManageReqCodeTextASCII / ManageReqCodeBinary / ManageReqCodeModbus
 *
 * ; do nothing, host can send it to DeviceAll to claim bus when some device's repsonse is out of time,
 * ; or send to a DeviceID to make sure it's responsive
 * ; corresponding MangeReqContent and MangeRespContent can be 0 length
 * ManageReqCodePing = %x00
 *
 * ; host give the bus to a specified device if the request device ID is not 0,
 * ; device can send the request to give back the bus to host is device ID is DeviceAll (0) (host does not need to answer coz. of using DeviceAll)
 * ; corresponding MangeReqContent and MangeRespContent can be 0 length
 * ManageReqCodeBus = %x01
 *
 * ; device should be reset
 * ; corresponding MangeReqContent and MangeRespContent can be 0 length
 * ManageReqCodeReset = %x02
 *
 * ; set device state, disabled or working state
 * ; corresponding MangeReqContent is MangeReqContentState
 * ; corresponding MangeRespContent can be 0 length
 * MangeReqContentState = DeviceState ; set the state of device
 * ManageReqCodeState = %x03
 *
 * ; host sends this to a DeviceID to query the status of a specified device
 * ; corresponding MangeReqContent can be 0 length
 * ; corresponding MangeRespContent is MangeRespContentDeviceStatus
 * ManageReqCodeDeviceStatus = %x04
 * MangeRespContentDeviceStatus = DeviceState *DeviceStatusCode ; device state code and concatenation of severl status codes
 * DeviceStatusCode = DeviceStatusError / DeviceStatusSignal
 * DeviceStatusError = DeviceStatusFatal / DeviceStatusError / DeviceStatusWarn / DeviceStatusInit /
 *                    DeviceStatusRough / DeviceStatusInstab /
 *                    DeviceStatusEnvFatal / DeviceStatusEnvError / DeviceStatusEnvWarn /
 *                    DeviceStatusMechFatal / DeviceStatusMechError / DeviceStatusMechWarn
 * DeviceStatusFatal = %x01 ; device is totally out of control
 * DeviceStatusError = %x02 ; error happens, not-working, may be it can be recovered by the device itself
 * DeviceStatusWarn = %x03 ; still working, but there's warning
 * DeviceStatusInit = %x04 ; not working, device is in initialization state
 * DeviceStatusRough = %x05 ; still working, but device meets accuracy problem
 * DeviceStatusInstab = %x06 ; still working, but device meets stability problem
 * DeviceStatusEnvFatal = %x07 ; not working, environment is totally unbearable
 * DeviceStatusEnvError = %x08 ; not working, device meet hard environmental problem, may be it can be recovered by the device itself
 * DeviceStatusEnvWarn = %x09 ; still working, but device meets environmental problem
 * DeviceStatusMechFatal = %x0A ; not working, mechanical parts are damaged
 * DeviceStatusMechError = %x0B ; not working, device meet hard mechanical problem, may be it can be recovered by the device itself
 * DeviceStatusMechWarn = %x0C ; still working, but device meets mechanical problem
 * DeviceStatusSignal = DeviceStatusHome / DeviceStatusUpLimit / DeviceStatusDownLimit
 * DeviceStatusHome = %x7F ; device gives HOME signal
 * DeviceStatusUpLimit = %x7E ; device gives Up-Limit signal
 * DeviceStatusDownLimit = %x7D ; device gives Down-Limit signal
 *
 * ; to get a specified device's parameter, like analog sampling, switch status, using paramter index code
 * ; definition of parameters is not a part of this protocol, it's decided by on the system scene
 * ; corresponding MangeReqContent is DeviceParamIndex
 * ; corresponding MangeRespContent is according to the paramter, it can be a concatenation of multiple parameters
 * ManageReqCodeDeviceGet = %x05
 * MangeReqContentDeviceGet = DeviceParamIndex
 * DeviceParamIndex = UINT14 ; paramter index code
 * MangeRespContentDeviceGet = Parameters
 *
 * ; to set parameter to device, using paramter index code
 * ; corresponding MangeReqContent is MangeReqContentDeviceSet
 * ; corresponding MangeRespContent can be 0 length
 * ManageReqCodeDeviceSet = %x06
 * MangeReqContentDeviceSet = DeviceParamIndex Parameters
 *
 * ; using 7 bit ASCII strings to communicate with device
 * ; corresponding MangeReqContent and MangeRespContent are 7bit ASCII text
 * ManageReqCodeTextASCII = %x7E
 * MangeReqContentTextASCII  = Text
 * MangeRespContentTextASCII = Text
 *
 * ; using binary frames to communicate with device
 * ; corresponding MangeReqContent and MangeRespContent are wrapped binary data
 * ManageReqCodeBinary = %x7D
 * MangeReqContentBinary  = Bolb
 * MangeRespContentBinary = Bolb
 *
 * ; using Modbus RTU to communicate with device
 * ; corresponding MangeReqContent and MangeRespContent are wrapped Modbus RTU
 * ManageReqCodeBinary = %x7C
 * MangeReqContentBinary  = Bolb
 * MangeRespContentBinary = Bolb
 *
 * ; this is motion control and query stream
 * ; as a stream type transmition, host should send addresse code in a frequency, to reduce the impact of occasional packet loss
 * StepStream = %d248 *( StepStreamSync / StepStreamAync / ( StepStreamQuery StepStreamStatus) )
 * StepStreamSync = %d0-80 ; the number between 0-80 is decoded in base 3, as 0000-2222, 4 bits correspond to 4 motion devices,
 *                         ; device which has ID 1 is in bit 0, device which has ID 2 is in bit 1, and so on.
 *                         ; every bit, 0 means no move, 1 means move forward, 2 means move backward
 * StepStreamAync = %d81-104 ; the number 81/82 means device which has ID 5 move forward/backward,
 *                           ; the number 83/84 means device which has ID 6 move forward/backward, and so on,
 *                           ; the number 103/104 means device which has ID 16 move forward/backward
 * StepStreamQuery = %d105-120 ; the number 105 needs device which has ID 1 to answer its status in StepStreamStatus,
 *                             ; the number 106 needs device which has ID 2 to answer its status in StepStreamStatus, and so on,
 *                             ; the number 120 needs device which has ID 16 to answer its status in StepStreamStatus
 * ; these status code are sent by devices, then host take back the bus immediately
 * StepStreamStatus = StepStreamStatusBad / StepStreamStatusOK /
 *                    StepStreamStatusHome / StepStreamStatusUpLimit / StepStreamStatusDownLimit
 *
 * StepStreamStatusBad = %d121  ; device is disabled, or there's error, or it does not play as motion device
 * StepStreamStatusOK = %d122 ; device is OK, and there's no signal
 * StepStreamStatusHome = %d123   ; device stopped, it has touched HOME switch (some device can also combine Up-Limit/Down-Limit here)
 * StepStreamStatusUpLimit = %d124   ; device stopped, it has touched Up-Limit switch
 * StepStreamStatusDownLimit = %d125   ; device stopped, it has touched Down-Limit switch
 *
 * ; AdvancedStepStream can indicate many devices to move synchronized, and move more steps, or correct occasional transmission errors
 * ; AdvancedStepStreamBytes contains fixed number of AdvancedStepStreamByte, each AdvancedStepStreamByte contains move instruction of 2 devices
 * ; and the bit 6 of AdvancedStepStreamByte indicates if there are more AdvancedStepStreamByte following, 1 means yes and 0 means no
 * ; bit 0-2 of first AdvancedStepStreamByte instructs device with ID 1 how to move, bit 3-5 instructs device with ID2,
 * ; bit 0-2 of sencod AdvancedStepStreamByte instructs device with ID 3, and so on
 * ; in the last AdvancedStepStreamByte, bit6 is 0, and only when this byte is received, all device start move together.
 * ; 3 bits move instruction is encoded/decoded using a special code book (01234567->01236745), to reduce the impact of transmission error
 * ; device will move according to position changing, and select the shorted direction.
 * ; for example, when position is changed from 3 to 6, there's 2 paths of opposite step direction, 3456 and 321076, the shorted 3456 should be selected by device
 * ; step offset has be limited from -3 to +3, for example, changing from 3 to 7 will cause confusion, maybe it's -4, also +4 is possible
 * ; if host limits it's position changing between -1 to 1 steps, there will be a good error correction mechanism.
 * ; one thing should be noticed, the initialized start position is set to 0 every time %d246 address is received (but no move)
 * ; as a stream type transmition, host should send addresse code in a frequency, to reduce the impact of occasional packet loss
 * AdvancedStepStream = %d246 *AdvancedStepStreamBytes
 * AdvancedStepStreamBytes = 1*AdvancedStepStreamByte
 * AdvancedStepStreamByte = %x00-7F
 *
 *  !!! following protocol is draft, and not implemented !!!
 *
 * ; MoveCmd uses parameters to control device movement, it doesn't care about every single step, only about the target and the speed
 * ; host may send move instruction slightly faster than the execution, so devices need to buffer/queue the instructions, FIFO
 * ; if the buffer in device is full, host can try it again after a while (in fact, host should known the time of schedule)
 * ; host should known that other move instructions in other address will detroy the buffer
 * MoveCmd = %d244 (DeviceID / DeviceAll) MoveRequest MoveResponse
 * MoveRequest = MoveReqCode MoveReqLength MoveReqContent
 * MoveResponse = MoveRespCode MoveRespLength MoveRespContent
 * MoveReqLength = UINT14 ; length of following MoveReqContent
 * MoveReqContent = Parameters
 * MoveRespLength =UINT14 ; length of following MoveRespContent
 * MoveRespContent = Parameters
 *
 * ; MoveRespCode is an UINT7, indicates the result of command
 * MoveRespCode = MoveRespCodeOK / MoveRespCodeFail / MoveRespCodeNoSup / MoveRespCodeFull / MoveRespCodeBadPro / MoveRespCodeBadOp
 * MoveRespCodeOK = %d0     ; operation OK
 * MoveRespCodeFail = %d1   ; tried, and failed
 * MoveRespCodeNoSup = %d2 ; this request is not supported
 * MoveRespCodeNoMem = %d3   ; buffer is full, cannot hold more move instructions
 * MoveRespCodeBadPro = %d4 ; the protocol of MangeReqLength and MangeReqContent seems to be bad
 * MoveRespCodeBadOp = %d5  ; the parameters in MangeReqContent if out of device's ability
 *
 * MoveReqCode = MoveReqCodeStatus / MoveReqCodeInterval
 *
 * ; query move status
 * ; corresponding MoveReqContent can be 0 length
 * ; corresponding MoveRespContent is MoveRespContentStatus
 * MoveReqCodeStatus = %x00
 * MoveRespContentStatus = DeviceTimeStamp MoveBufferLeft MovePosition DeviceState *DeviceStatusCode
 * DeviceTimeStamp = UINT28 ; rolling time stamp of device
 * MoveBufferLeft = UINT28 ; how many move instruction buffer left, in bytes, device just stores each MoveReqLength and MoveReqContent
 * MovePosition = INT28 ; current rolling position in device
 *
 * ; move steps using time interval as instruction
 * ; corresponding MoveReqContent is MoveReqContentInterval
 * ; corresponding MoveRespContent is MoveRespContentInterval
 * MoveReqCodeInterval = %x02
 * MoveReqContentInterval = MoveOffset MoveInterval MoveIntervalAdd
 * MoveOffset = INT14; indicate the offset the move should take, unit is normally "step"
 * MoveInterval = UINT14 ; indicates time interval of each move step, the unit of it is defined by device, for example, MCU clocks
 * MoveIntervalAdd = INT14 ; after every step, time interval will be added by this value
 * MoveRespContentInterval = MoveBufferLeft
 *
 * ; this will sampling the position using device's position encoder
 * ; device should have a buffer to store position points and return data to host in batch mode
 * ; host should already known which device can do sampling, how many points can be stored by devices, to plan its schedule
 * ; devices which do not support sampling, just ignore all data in this communication address
 * Sampling = %d238 ( SamplingStop / SamplingGet / SamplingStartOnTime / SamplingStartOnMove )
 *
 * ; this instructs devices to stop current sampling, release it buffer, devices do not need to answer this request
 * SamplingStop = %x00 (DeviceID / DeviceAll)
 *
 * ; this instructs device to output sampling buffer to the bus, and make room for further sampling, but does not stop sampling
 * ; content in buffer is different for each SamplingStartXXX instruction
 * SamplingGet = %x01 DeviceID SamplingResponseCode SamplingBufferLength SamplingBuffer
 * SamplingBufferLength = UINT14 ; sent by device, length of following SamplingBuffer
 * SamplingBuffer = SamplingBufferOnTime / SamplingBufferOnMove

 * ; this is send by device, to indicate success or failure
 * SamplingResponseCode = SamplingResponseOK / SamplingResponseNotSupp / SamplingResponseFail / SamplingResponseNoMem / SamplingResponseMiss
 * SamplingResponseOK = %x00  ; no problem
 * SamplingResponseNotSupp = %x01 ; device does not support such operation
 * SamplingResponseFail = %x02 ; operation failed
 * SamplingResponseNoMem = %x03 ; the buffer schedule planed by host is not reachable
 * SamplingResponseMiss = %x04 ; given when SamplingGet, to indicate that some points are missed, maybe it's because of out of buffer size
 *
 * ; this instruct device to start sampling on fixed interval of time, using "step" as position unit
 * ; devices do not need to answer this request
 * ; corresponding SamplingBuffer is SamplingBufferOnTime
 * SamplingStartOnTime = %x02 (DeviceID / DeviceAll) SamplingTimeInterval
 * SamplingTimeInterval = UINT28 ;
 * SamplingBufferOnTime = SamplingPointTime *SamplingPointPos
 * SamplingPointTime = UINT28 ; the cyclic rolling timestamp of first point, in host timestamp unit, start from 0 when SamplingStartOnTime is got
 * SamplingPointPos = INT28 ; position in step unit, 0 means the posistion when SamplingStartOnTime is got
 *
 * ; this instruct device to start sampling, and store position only when the travel distance exceeds the position interval given
 * ; in fact, devicees can decide on a better resolution and algorithm based on the position interval given
 * ; devices do not need to answer this request
 * ; corresponding SamplingBuffer is SamplingBufferOnMove
 * SamplingStartOnMove = %x03 (DeviceID / DeviceAll) SamplingPosInterval
 * SamplingPosInterval = UINT14 ; absolute interval in "step"
 * SamplingBufferOnMove = *(SamplingPointTimePart SamplingPointPosPart)
 * SamplingPointTimePart = UINT14 ; time elapsed value relative to the last time
 * SamplingPointPosPart = INT14 ; posistion changing value relative to the last position
 *
 * not all of above are implemented
 *
 *  Created on: 2023.11.6
 *      Author: johnny
 */

#include <motion_foc.h>
#include "inputter.h"
#include "flash.h"
#include "motion.h"
#include "motion_track.h"
#include "driver.h"
#include "encoder.h"
#include "env.h"

#include "air001xx_ll_bus.h"
#include "air001xx_ll_system.h"
#include "air001xx_ll_gpio.h"
#include "air001xx_ll_usart.h"
#include "air001xx_ll_dma.h"

#include <stdio.h>
#include <stdarg.h>

#ifdef INPUTTER_RS485

static volatile int32_t  value;

typedef struct {
  uint32_t id;            // should be 0~127, 0 means default
  uint32_t baudrate;      // baudrate
} params_t;

static const params_t params_def={
    .id=1,
    .baudrate=CONF_RS485_DEF_BAUDRATE,
};

static const volatile params_t __attribute__((used,__aligned__(4),section(".inputter_param"))) params_flash=params_def;  // initialized as default, and so that volatile is needed

static __attribute__((__aligned__(4))) params_t params; // a used version of params_flash, no need to be volatile, for IRQ handler, it will not be optimized

#define MOVE_BUFF_SIZE  256
static  uint8_t  move_buff[MOVE_BUFF_SIZE];
static  uint32_t move_head;
static  uint32_t move_tail;
static  uint64_t move_tick;

#define RX_BUFF_SIZE       256 /* for 250kbps, it can hold 10ms data */
static  uint8_t            rx_buff [RX_BUFF_SIZE];
static  uint32_t           rx_head;

#define COMM_BUFF_SIZE  128

typedef enum {
  COMM_DISABLE=0,
  COMM_ENABLE=1,
  COMM_MOTION_TRK=0x10,
  COMM_MOTION_FOC=0x11,
} comm_state_t;

// non-volatile, coz of all of this is process by a single function and its sub-function
static comm_state_t      comm_state;
static uint8_t           comm_addr;
static uint32_t          comm_stage;
static uint8_t           comm_buff[COMM_BUFF_SIZE];

static volatile uint8_t  comm_ascii;

#define COMM_STATE_IN_MOTION (comm_state==COMM_MOTION_TRK || comm_state==COMM_MOTION_FOC)

// for step stream conversion
static const uint8_t tab_step_stream_decode[81]={
    0,1,2,4,5,6,8,9,10,16,17,18,20,21,22,24,25,26,32,33,34,36,37,38,40,41,42,64,
    65,66,68,69,70,72,73,74,80,81,82,84,85,86,88,89,90,96,97,98,100,101,102,
    104,105,106,128,129,130,132,133,134,136,137,138,144,145,146,148,149,
    150,152,153,154,160,161,162,164,165,166,168,169,170};

static const uint8_t tab_step_stream_shift[5]={0,0,2,4,6};

static const uint8_t tab_advstep_stream_decode[8]={0,1,2,3,6,7,4,5};

#define DIR_RX      LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);
#define DIR_TX      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3);

APP_REG_ERROR(static, err_emergency, "Emergency")

static void init(void)
{
  // get from flash
  memcpy(&params, (const void*)&params_flash, sizeof(params_t));

  // USART2
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_USART2);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_USART2);

  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_SetHWFlowCtrl(USART2, LL_USART_HWCONTROL_NONE);
  LL_USART_SetBaudRate(USART2, APP_CLK_APB1, LL_USART_OVERSAMPLING_16, params.baudrate); // there's a bug with LL_USART_OVERSAMPLING_8, AIR002 does not need to use fck*2 when using LL_USART_OVERSAMPLING_8, but the LL function does
  LL_USART_SetOverSampling(USART1, LL_USART_OVERSAMPLING_16);
  LL_USART_ConfigCharacter(USART2, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
  LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_RX); // initalized RX
  LL_USART_EnableDMAReq_RX(USART2);
  LL_USART_EnableIT_RXNE(USART2);

  // DMA for RX (CH2)
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_2,
                      LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                      LL_DMA_MODE_CIRCULAR |
                      LL_DMA_PERIPH_NOINCREMENT  |
                      LL_DMA_MEMORY_INCREMENT  |
                      LL_DMA_PDATAALIGN_BYTE |
                      LL_DMA_MDATAALIGN_BYTE |
                      LL_DMA_PRIORITY_VERYHIGH);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&(USART2->DR));
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_VERYHIGH);

  // DMA remap
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_SYSCFG_SetDMARemap_CH2(LL_SYSCFG_DMA_MAP_USART2_RX);

  // PA0:USART2_TX PA3:USART2_RX PB3:RS485 DIR
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA|LL_IOP_GRP1_PERIPH_GPIOB);

  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_3, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  DIR_RX // init dir
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_0, LL_GPIO_AF9_USART2);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);

  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_UP); // useful when driver output is Z
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF4_USART2);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);

  // priority must be lower than systick and driver and sampler
  NVIC_SetPriority(USART2_IRQn, 2);
  NVIC_EnableIRQ(USART2_IRQn);

  value=0;

  move_head=0;
  move_tail=0;
  move_tick=app_cnt_clk();

  rx_head=0;
  comm_state=COMM_ENABLE; // DeviceStateEnable
  comm_addr=0;
  comm_ascii=0;

  // start listening
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)rx_buff);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, RX_BUFF_SIZE);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_USART_Enable(USART2);
}

int32_t  inputter_get(void)
{
  if (move_head!=move_tail) {


  }
  return value;
}

static const char * comm_state_str(void)
{
  switch(comm_state) {
    case COMM_DISABLE: return "Disable";
    case COMM_ENABLE: return "Enable";
    case COMM_MOTION_TRK: return "Track";
    case COMM_MOTION_FOC: return "FOC";
    default: return "Unknown";
  }
}

static void cmd(char * cmdstr, app_cmd_res_t resp)
{
  char *str=strtok_r(cmdstr, " \t\r\n", &cmdstr);

  if (!str || !*str)
    app_resp_printf(resp, "RS485: %s, POS=%ld, ID=%ld, BAUD=%lu\r\n", comm_state_str(), value, params.id, params.baudrate);
  else if (strcasecmp("set", str)==0) {
    value=app_sscan_int(&cmdstr, value, -2147483648, 2147483647);
    app_resp_printf(resp, "RS485: POS=%ld\r\n", value);
  }
  else if (strcasecmp("add", str)==0) {
    app_irq_disable();
    value+=app_sscan_int(&cmdstr, 0, -2147483648, 2147483647);
    app_irq_resume();
    app_resp_printf(resp, "RS485: POS=%ld\r\n", value);
  }
  else if (strcasecmp("conf", str)==0) {
    params.id=(uint32_t)app_sscan_int(&cmdstr, (int32_t)params.id, 0, 80);
    params.baudrate=(uint32_t)app_sscan_int(&cmdstr, (int32_t)params.baudrate, 9600, 3000000);
    // change hardware baudrate
    LL_USART_Disable(USART2);
    LL_USART_SetBaudRate(USART2, APP_CLK_APB1, LL_USART_OVERSAMPLING_16, params.baudrate); // there's a bug with LL_USART_OVERSAMPLING_8, AIR002 does not need to use fck*2 when using LL_USART_OVERSAMPLING_8, but the LL function does
    LL_USART_Enable(USART2);
    // burn to FLASH
    if (flash_write((void*)&params_flash, &params, sizeof(params_t))) {
      app_resp_printf(resp, "RS485: Configuration is saved to FLASH\r\n");
    } else
      app_resp_printf(resp, "RS485: Failed to write FLASH\r\n");
  }
  else
    app_resp_printf(resp, "RS485: <set [pos:+-2^31]> <add [pos:+-2^31]> <conf [id:0-80] [baud:9600-3000000]>\r\n");
}

static void send(const uint8_t *data, uint32_t len)
{
  DIR_TX
  LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX);
  while (len--) {
    while (!LL_USART_IsActiveFlag_TXE(USART2));
    LL_USART_TransmitData8(USART2, *(data++));
  }
  while (!LL_USART_IsActiveFlag_TC(USART2));
  DIR_RX
  LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_RX);
}

static void resp_send (const char * str, uint32_t len)
{
  if (comm_ascii && comm_buff[0]) {
    uint8_t head[3]={1, (len>>7)&0x7F, (len)&0x7F};
    send(head, 3);
    send((const uint8_t *)str, len);
  }
}

static inline __attribute__((always_inline)) uint32_t param_uint28(uint8_t * data)
{
  return ((uint32_t)data[0]<<21)|((uint32_t)data[1]<<14)|((uint32_t)data[2]<<7)|((uint32_t)data[3]);
}

static inline __attribute__((always_inline)) uint32_t param_uint14(uint8_t * data)
{
  return ((uint32_t)data[0]<<7)|((uint32_t)data[1]);
}

static inline __attribute__((always_inline)) void proc_Manage(uint8_t data)
{
  if (comm_stage<3) { // DeviceID, ManageReqCode, high byte of MangeReqLength
    if (comm_stage==0) {
      if (data && data!=params.id) { // not my bussiness
        comm_addr=0;
        return;
      }
    }
    comm_buff[comm_stage++]=data;
    return;
  } else { // MangeReqLength_low_part, MangeReqContent
    if (comm_stage<COMM_BUFF_SIZE) // drop overflow data
      comm_buff[comm_stage++]=data;
    else
      comm_stage++;
    if (comm_stage<param_uint14(comm_buff+2)+4)
      return;
  }

  comm_addr=0; // it must, and will be done

  switch(comm_buff[1]) {
  case 0: // ManageReqCodePing
    if (comm_buff[0])
      send((const uint8_t*)"\0\0\0", 3); // MangeRespCodeOK
    break;
  case 2: // ManageReqCodeReset
    if (comm_buff[0])
      send((const uint8_t*)"\0\0\0", 3); // MangeRespCodeOK
    app_reset(); // this will never return
    break;
  case 3: // ManageReqCodeState
    if (param_uint14(comm_buff+2)>0) {
      if (comm_buff[4]==COMM_MOTION_TRK) { // DeviceStateMotion0
        comm_state=comm_buff[4];
        motion_track_enable();
        if (comm_buff[0])
          send((const uint8_t*)"\0\0\0", 3); // MangeRespCodeOK
      } else if (comm_buff[4]==COMM_MOTION_FOC) { // DeviceStateMotion1
        comm_state=comm_buff[4];
        motion_foc_enable();
        if (comm_buff[0])
          send((const uint8_t*)"\0\0\0", 3); // MangeRespCodeOK
      } else if (comm_buff[4]==COMM_ENABLE || comm_buff[4]==COMM_DISABLE) { // DeviceStateEnable / DeviceStateDisable
        if (comm_state==COMM_MOTION_TRK)
          motion_track_disable();
        else if (comm_state==COMM_MOTION_FOC)
          motion_foc_disable();
        comm_state=comm_buff[4];
        if (comm_buff[0])
          send((const uint8_t*)"\0\0\0", 3); // MangeRespCodeOK
      } else {
        if (comm_buff[0])
          send((const uint8_t*)"\x06\0\0", 3); // MangeRespCodeBadOp
      }
    } else if (comm_buff[0])
      send((const uint8_t*)"\x05\0\0", 3); // MangeRespCodeBadPro
    break;
  case 4: // ManageReqCodeDeviceStatus
    if (comm_buff[0]) {
      comm_buff[0]=0; // MangeRespCodeOK
      comm_buff[1]=0;
      comm_buff[3]=comm_state;
      comm_stage=4;
      if (app_error&APP_ERROR_FATAL)
        comm_buff[comm_stage++]=0x01; // DeviceStatusFatal
      if (app_error&env_err_high_temp)
        comm_buff[comm_stage++]=0x07; // DeviceStatusEnvError
      if (app_error&~(env_err_high_temp|APP_ERROR_FATAL))
        comm_buff[comm_stage++]=0x02; // DeviceStatusError
      motion_state_t _motion_state=motion_state;
      if (_motion_state==MOTION_HOME)
        comm_buff[comm_stage++]=0x7F; // DeviceStatusHome
      else if (_motion_state==MOTION_LIMIT_UP)
        comm_buff[comm_stage++]=0x7E; // DeviceStatusUpLimit
      else if (_motion_state==MOTION_LIMIT_DOWN)
        comm_buff[comm_stage++]=0x7D; // DeviceStatusDownLimit
      else if (_motion_state==MOTION_BLOCKED)
        comm_buff[comm_stage++]=0x0C; // DeviceStatusMechWarn
      else if (_motion_state==MOTION_LOST)
        comm_buff[comm_stage++]=0x0B; // DeviceStatusMechError
      comm_buff[2]=(uint8_t)(comm_stage-3);
      send(comm_buff, comm_stage);
    }
    break;
  case 5: // ManageReqCodeDeviceGet
  case 6: // ManageReqCodeDeviceSet
    if (comm_buff[0])
      send((const uint8_t*)"\x06\0\0", 3); // MangeRespCodeBadOp
    break;
  case 126: // ManageReqCodeTextASCII
    if (comm_stage<COMM_BUFF_SIZE)
      comm_buff[comm_stage]=0;
    else
      comm_buff[COMM_BUFF_SIZE-1]=0;
    comm_ascii=1;
    break;
  default:
    if (comm_buff[0])
      send((const uint8_t*)"\x04\0\0", 3); // MangeRespCodeNoSup
    break;
  }
}

static inline __attribute__((always_inline)) void proc_StepStream(uint8_t data)
{
  if (data<=80) { // StepStreamSync
    if (params.id<=4 && COMM_STATE_IN_MOTION) {
      uint8_t step=tab_step_stream_decode[data]>>tab_step_stream_shift[params.id];
      if (step & 0x01) {
        value++;
      } else if (step & 0x02) {
        value--;
      }
    }
  } else if (data<=104) { // StepStreamAync
    if (params.id>4 && COMM_STATE_IN_MOTION) {
      uint32_t c=(params.id<<1)+71U;
      if ((uint32_t)data==c) {
        value++;
      } else if ((uint32_t)data==c+1U) {
        value--;
      }
    }
  } else if (data<=120) { // StepStreamQuery
    if ((uint32_t)(data-104)==params.id) {
      if (app_error || !COMM_STATE_IN_MOTION)
        send((const uint8_t *)"\x79",1); // StepStreamStatusBad
      else {
        motion_state_t _motion_state=motion_state;
        if (_motion_state==MOTION_HOME)
          send((const uint8_t *)"\x7B",1); // StepStreamStatusHome
        else if (_motion_state==MOTION_LIMIT_UP)
          send((const uint8_t *)"\x7C",1); // StepStreamStatusUpLimit
        else if (_motion_state==MOTION_LIMIT_DOWN)
          send((const uint8_t *)"\x7D",1); // StepStreamStatusDownLimit
        else
          send((const uint8_t *)"\x7A",1); // StepStreamStatusOK
      }
    }
  }
}

static inline __attribute__((always_inline)) void proc_AdvancedStepStream(uint8_t data)
{
  if (comm_stage+1==params.id) {
    comm_buff[1]=tab_advstep_stream_decode[data&0x07];
  } else if (comm_stage+2==params.id) {
    comm_buff[1]=tab_advstep_stream_decode[(data>>3)&0x07];
  }

  if (data & 0x40)
    comm_stage++;
  else {
    comm_stage=0;
    if (comm_buff[1]>comm_buff[0]) {
      if (comm_buff[1]<=comm_buff[0]+3) {
        value+=comm_buff[1]-comm_buff[0];
      } else if (comm_buff[0]+5<=comm_buff[1]) {
        value-=comm_buff[0]+8-comm_buff[1];
      }
    } else if (comm_buff[1]<comm_buff[0]) {
      if (comm_buff[0]<=comm_buff[1]+3) {
        value-=comm_buff[0]-comm_buff[1];
      } else if (comm_buff[1]+5<=comm_buff[0]) {
        value+=comm_buff[1]+8-comm_buff[0];
      }
    }
    comm_buff[0]=comm_buff[1];
  }
}

static inline __attribute__((always_inline)) void proc_MoveStream(uint8_t data)
{
  move_buff[move_head]=data;
  move_head++;
  if (move_head>=MOVE_BUFF_SIZE)
    move_head=0;

}

static inline __attribute__((always_inline)) void proc_Sampling(uint8_t data)
{
  (void)data; // not implemented


}

static inline __attribute__((always_inline)) void proc_data(uint8_t data)
{
  if (data==254) { // emergency stop
    app_irq_disable();
    app_error|=err_emergency;
    app_irq_resume();
    motion_disable();
    comm_addr=0;
    comm_ascii=0;
    return;
  }

  // correct current state
  if (COMM_STATE_IN_MOTION && motion_state==MOTION_DISABLE)
    comm_state=COMM_ENABLE; // not in motion state, just DeviceStateEnable

  if (data&0x80) { // an address
    comm_stage=0;
    comm_addr=data;
    comm_ascii=0;
    if (app_error & err_emergency) {
      app_irq_disable();
      app_error&=~err_emergency;
      app_irq_resume();
    }
    if (!params.id) {
        comm_addr=0; // not my bussiness
    } else if (comm_addr==248) {  // StepStream
      if (params.id>16)
        comm_addr=0; // not my bussiness
    } else if (comm_addr==246) { // AdvancedStepStream
      if (!COMM_STATE_IN_MOTION)
        comm_addr=0; // not my bussiness
      else {
        comm_buff[0]=0; // used as old position
        comm_buff[1]=0; // used as new position
      }
    } else if (comm_addr==244) { // MoveStream
      if (!COMM_STATE_IN_MOTION)
        comm_addr=0; // not my bussiness
    }
    return;
  }

  if (comm_addr==248) // StepStream
    proc_StepStream(data);
  else if (comm_addr==246) // AdvancedStepStream
    proc_AdvancedStepStream(data);
  else if (comm_addr==244) // MoveStream
    proc_MoveStream(data);
  else if (comm_addr==238) // Sampling
    proc_Sampling(data);
  else if (comm_addr==253) // Manage
    proc_Manage(data);
}

void __attribute((interrupt("IRQ")))  USART2_IRQHandler(void)
{
  if (USART2->SR & USART_SR_RXNE) { // in fact, DMA has cleaned it
    uint32_t dr=USART2->DR; (void)dr; // read and clear RXNE/IDLE/ORE/NE/PE/FE
  }

  while(rx_head!=RX_BUFF_SIZE-LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_2)) {
    proc_data(rx_buff[rx_head]);
    if ((++rx_head)>=RX_BUFF_SIZE)
      rx_head=0;
  }
}

static void loop(void)
{
  if (comm_ascii) {
    app_printf("RS485: CMD=\"%s\"\r\n", comm_buff+3);
    app_cmd((char*)comm_buff+4, &resp_send); // todo: it's rough implement, might be delayed, and can't abort
    if (comm_ascii && comm_buff[0])
      send((const uint8_t*)"\0\0\0", 3); // MangeRespCodeOK
    comm_ascii=0;
  }
}

APP_REG_SUBMODULE("inp", &init, &loop, &cmd, "RS485 interface")

#endif
