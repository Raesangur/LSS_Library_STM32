/**
 *	Author:			Sebastien Parent-Charette (support@robotshop.com)
 *	Version:		1.2.0
 *	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
 *	
 *	Description:	A library that makes using the LSS simple.
 *					Offers support for both Arduino Uno, Mega and others through
 *					the use of the Stream class for communication.
 *
 *  Modified By:    Pascal-Emmanuel Lachance (raesangur.com)
 *  Modification:   Rewritten the library to work with the STM32's HAL
 */
/*************************************************************************************************/
/* File includes ------------------------------------------------------------------------------- */
#include "LSS.h"


/*************************************************************************************************/
/* Macros ----------------------- -------------------------------------------------------------- */
#define CHECK_COMM_STATUS(lss, Query, ReturnValue)                                                \
	do                                                                                            \
	{                                                                                             \
		if (!generic_write(lss, Query))                                                           \
		{                                                                                         \
			lss->lastCommStatus = LSS_CommStatus_WriteNoBus;                                      \
			return ReturnValue;                                                                   \
		}                                                                                         \
	} while (0)

#define CHECK_COMM_STATUS_TYPE(lss, Query, ReturnValue, Type)                                     \
	do                                                                                            \
	{                                                                                             \
		if (!generic_write_val(lss, Query, Type))                                                 \
		{                                                                                         \
			lss->lastCommStatus = LSS_CommStatus_WriteNoBus;                                      \
			return ReturnValue;                                                                   \
		}                                                                                         \
	} while (0)


/*************************************************************************************************/
/* Constants ----------------------------------------------------------------------------------- */
#define LSS_SupportsSettingTimeouts

//> Bus communication
#define  LSS_DEFAULT_BAUD               (115200)
#define LSS_TIMEOUT				        100		// in ms
#define LSS_COMMAND_START		        ("#")
#define LSS_COMMAND_REPLY_START	        ("*")
#define LSS_COMMAND_END			        ("\r")
#define LSS_FIRST_POSITION_DISABLED	    ("DIS")
#define LSS_MAX_TOTAL_COMMAND_LENGTH    (30 + 1)   // ex: #999XXXX-2147483648\r; Adding 1 for end string char (\0)
									            // ex: #999XX000000000000000000\r;

//> Servo constants
#define LSS_ID_DEFAULT				(0)
#define LSS_ID_MIN					(0)
#define LSS_ID_MAX					(250)
#define LSS_MODE_255ID				(255)
#define LSS_BROADCAST_ID			(254)

#define LSS_MODEL_HT1	            "LSS-HT1"
#define LSS_MODEL_ST1	            "LSS-ST1"
#define LSS_MODEL_HS1	            "LSS-HS1"

//> Commands - actions
#define LSS_ACTION_RESET				    ("RESET")
#define LSS_ACTION_LIMP				        ("L")
#define LSS_ACTION_HOLD		                ("H")
#define LSS_ACTION_PARAMETER_TIME           ("T")
#define LSS_ACTION_PARAMETER_CURRENT_HOLD   ("CH")
#define LSS_ACTION_PARAMETER_SPEED	        ("S")
#define LSS_ACTION_MOVE				        ("D")
#define LSS_ACTION_MOVE_RELATIVE		    ("MD")
#define LSS_ACTION_WHEEL				    ("WD")
#define LSS_ACTION_WHEEL_RPM			    ("WR")

//> Commands - actions (settings)
#define LSS_ACTION_ORIGIN_OFFSET		("O")
#define LSS_ACTION_ANGULAR_RANGE		("AR")
#define LSS_ACTION_MAX_SPEED			("SD")
#define LSS_ACTION_MAX_SPEED_RPM		("SR")
#define LSS_ACTION_COLOR_LED			("LED")
#define LSS_ACTION_GYRE_DIRECTION		("G")

//> Commands - actions (advanced settings)
#define LSS_ACTION_ANGULAR_STIFFNESS			("AS")
#define LSS_ACTION_ANGULAR_HOLDING_STIFFNESS	("AH")
#define LSS_ACTION_ANGULAR_ACCELERATION		    ("AA")
#define LSS_ACTION_ANGULAR_DECELERATION		    ("AD")
#define LSS_ACTION_ENABLE_MOTION_CONTROL		("EM")
#define LSS_FILTER_POSITION_COUNT				("FPC")

//> Commands - queries
#define LSS_QUERY_STATUS			("Q")
#define LSS_QUERY_ORIGIN_OFFSET		("QO")
#define LSS_QUERY_ANGULAR_RANGE		("QAR")
#define LSS_QUERY_POSITION_PULSE	("QP")
#define LSS_QUERY_POSITION			("QD")
#define LSS_QUERY_SPEED				("QWD")
#define LSS_QUERY_SPEED_RPM			("QWR")
#define LSS_QUERY_SPEED_PULSE		("QS")
#define LSS_QUERY_MAX_SPEED			("QSD")
#define LSS_QUERY_MAX_SPEED_RPM		("QSR")
#define LSS_QUERY_COLOR_LED			("QLED")
#define LSS_QUERY_GYRE				("QG")
#define LSS_QUERY_ID				("QID")
#define LSS_QUERY_BAUD				("QB")
#define LSS_QUERY_FIRST_POSITION	("QFD")
#define LSS_QUERY_MODEL_STRING		("QMS")
#define LSS_QUERY_SERIAL_NUMBER		("QN")
#define LSS_QUERY_FIRMWARE_VERSION	("QF")
#define LSS_QUERY_VOLTAGE			("QV")
#define LSS_QUERY_TEMPERATURE		("QT")
#define LSS_QUERY_CURRENT			("QC")
#define LSS_QUERY_ANALOG			("QA")

//> Commands - queries (advanced)
#define LSS_QUERY_ANGULAR_STIFFNESS			("QAS")
#define LSS_QUERY_ANGULAR_HOLDING_STIFFNESS	("QAH")
#define LSS_QUERY_ANGULAR_ACCELERATION		("QAA")
#define LSS_QUERY_ANGULAR_DECELERATION		("QAD")
#define LSS_QUERY_ENABLE_MOTION_CONTROL		("QEM")
#define LSS_QUERY_FILTER_POSITION_COUNT		("QFPC")
#define LSS_QUERY_BLINKING_LED				("QLB")

//> Commands - configurations
#define LSS_CONFIG_ID						("CID")
#define LSS_CONFIG_BAUD						("CB")
#define LSS_CONFIG_ORIGIN_OFFSET			("CO")
#define LSS_CONFIG_ANGULAR_RANGE			("CAR")
#define LSS_CONFIG_MAX_SPEED				("CSD")
#define LSS_CONFIG_MAX_SPEED_RPM			("CSR")
#define LSS_CONFIG_COLOR_LED				("CLED")
#define LSS_CONFIG_GYRE_DIRECTION			("CG")
#define LSS_CONFIG_FIRST_POSITION			("CFD")
#define LSS_CONFIG_MODE_RC					("CRC")
#define LSS_CONFIG_FILTER_POSITION_CURRENT	("CFPC")

//> Commands - configurations (advanced)
#define LSS_CONFIG_ANGULAR_STIFFNESS			("CAS")
#define LSS_CONFIG_ANGULAR_HOLDING_STIFFNESS	("CAH")
#define LSS_CONFIG_ANGULAR_ACCELERATION		    ("CAA")
#define LSS_CONFIG_ANGULAR_DECELERATION		    ("CAD")
#define LSS_CONFIG_BLINKING_LED				    ("CLB")


/*************************************************************************************************/
/* Private functions declarations -------------------------------------------------------------- */
static bool     set_session_config     (LSS* lss, LSS_SetType setType, int16_t value,
										const char* sessionAction, const char* configAction);

static int16_t  timed_read             (LSS* lss);
static void     set_read_timeouts      (LSS* lss, uint32_t startResponseTimeout,
                                        uint32_t msgCharTimeout);

static void     init_bus               (LSS* lss, UART_HandleTypeDef* huart, uint32_t baud);
static void     close_bus              (LSS* lss);

static bool     generic_write          (LSS* lss, const char* cmd);
static bool     generic_write_val      (LSS* lss, const char* cmd, int16_t value);
static bool     generic_write_val_param(LSS* lss, const char* cmd, int16_t value,
                                        const char* parameter, int16_t parameterValue);

static uint16_t generic_read_s16       (LSS* lss, const char* cmd);
static char*    generic_read_str       (LSS* lss, const char* cmd);


/*********************************************************************************************/
/* Public functions definitions ------------------------------------------------------------ */

/* ----------- */
/* Constructor */
void LSS(LSS* lss, uint8_t id, UART_HandleTypeDef* huart, uint32_t baud)
{
    assert_param(id > LSS_ID_MIN && id < LSS_ID_MAX);
    
	/* Init id */
	lss->servoID = id;

	/* Init bus */
	init_bus(lss, huart, baud);
}


/* ------- */
/* Actions */

// Note: no waiting is done here. LSS will take a bit more than a second to reset/start responding to commands.
bool reset(LSS* lss)
{
	return generic_write(lss, LSS_ACTION_RESET);
}

bool limp(LSS* lss)
{
	return generic_write(lss, LSS_ACTION_LIMP);
}

// Make LSS hold current position
bool hold(LSS* lss)
{
	return generic_write(lss, LSS_ACTION_HOLD);
}

// Make LSS move to specified position in 1/10°
bool move(LSS* lss, int16_t value)
{
	return generic_write_val(lss, LSS_ACTION_MOVE, value);
}

// Make LSS move to specified position in 1/10° with T parameter
bool move_t(LSS* lss, int16_t value, int16_t tValue)
{
	return generic_write_val_param(lss, LSS_ACTION_MOVE, value, LSS_ACTION_PARAMETER_TIME, tValue);
}

// Make LSS move to specified position in 1/10° with CH parameter
bool move_ch(LSS* lss, int16_t value, int16_t chValue)
{
	return generic_write_val_param(lss, LSS_ACTION_MOVE, value, LSS_ACTION_PARAMETER_CURRENT_HOLD, chValue);
}

// Perform relative move by specified amount of 1/10°
bool move_relative(LSS* lss, int16_t value)
{
	return generic_write_val(lss, LSS_ACTION_MOVE_RELATIVE, value);
}

// Perform relative move by specified amount of 1/10° with T parameter
bool move_relative_t(LSS* lss, int16_t value, int16_t tValue)
{
	return generic_write_val_param(lss, LSS_ACTION_MOVE_RELATIVE, value, LSS_ACTION_PARAMETER_TIME, tValue);
}

// Make LSS rotate at set speed in (1/10°)/s
bool wheel(LSS* lss, int16_t value)
{
	return generic_write_val(lss, LSS_ACTION_WHEEL, value);
}

// Make LSS rotate at set speed in RPM
bool wheel_rpm(LSS* lss, int8_t value)
{
	return generic_write_val(lss, LSS_ACTION_WHEEL_RPM, value);
}


/* ------- */
/* Queries */

// Returns current status
LSS_Status get_status(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_STATUS, LSS_StatusUnknown);
	return (LSS_Status) generic_read_s16(lss, LSS_QUERY_STATUS);
}

// Returns origin offset in 1/10°
int16_t get_origin_offset(LSS* lss, LSS_QueryType queryType)
{
	CHECK_COMM_STATUS_TYPE(lss, LSS_QUERY_ORIGIN_OFFSET, 0, queryType);
	return (int16_t) generic_read_s16(lss, LSS_QUERY_ORIGIN_OFFSET);
}

// Returns angular range in 1/10°
uint16_t get_angular_range(LSS* lss, LSS_QueryType queryType)
{
	CHECK_COMM_STATUS_TYPE(lss, LSS_QUERY_ANGULAR_RANGE, 0, queryType);
	return generic_read_s16(lss, LSS_QUERY_ANGULAR_RANGE);
}

// Returns position in µs pulses (RC style)
uint16_t get_position_pulse(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_POSITION_PULSE, 0);
	return generic_read_s16(lss, LSS_QUERY_POSITION_PULSE);
}

// Returns position in 1/10°
int32_t get_position(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_POSITION, 0);

	// Read response from servo (as string)
	char* valueStr = generic_read_str(lss, LSS_QUERY_POSITION);
	int32_t valuePos = 0;

	// Check for disabled first position
	if (str_to_int(valueStr, &valuePos))
	{
		return valuePos;
	}
	else
	{
		// Unable to convert valueStr to int16_t
		return 0;
	}
}

int16_t get_first_position(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_FIRST_POSITION, 0);

	// Read response from servo (as string)
	char* valueStr = generic_read_str(lss, LSS_QUERY_FIRST_POSITION);

	// Check for disabled first position
	if (strcmp(valueStr, LSS_FIRST_POSITION_DISABLED) == 0)
	{
		// First position is not defined - invalid
		return 0;
	}
	else
	{
		int32_t valuePos = 0;
		if (str_to_int(valueStr, &valuePos))
		{
			return (int16_t)valuePos;
		}
		else
		{
			// Unable to convert valueStr to int16_t
			return 0;
		}
	}
}

bool get_is_first_position_enabled(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_FIRST_POSITION, 0);

	// Read response from servo (as string)
	char* valueStr = generic_read_str(lss, LSS_QUERY_FIRST_POSITION);
	if (lss->lastCommStatus == LSS_CommStatus_ReadSuccess)
	{
		// Check if first position is disabled
		return strcmp(valueStr, LSS_FIRST_POSITION_DISABLED);
	}
	else
	{
		// Read was not completed;
		return false;
	}
}


// Returns speed in (1/10°)/s
int16_t get_speed(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_SPEED, 0);
	return (int16_t) generic_read_s16(lss, LSS_QUERY_SPEED);
}

int8_t get_speed_rpm(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_SPEED_RPM, 0);
	return (int8_t)generic_read_s16(lss, LSS_QUERY_SPEED_RPM);
}

int8_t get_speed_pulse(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_SPEED_PULSE, 0);
	return (int8_t)generic_read_s16(lss, LSS_QUERY_SPEED_PULSE);
}

uint16_t get_max_speed(LSS* lss, LSS_QueryType queryType)
{
	CHECK_COMM_STATUS_TYPE(lss, LSS_QUERY_MAX_SPEED, 0, queryType);
	return generic_read_s16(lss, LSS_QUERY_MAX_SPEED);
}

int8_t get_max_speed_rpm(LSS* lss, LSS_QueryType queryType)
{
	CHECK_COMM_STATUS_TYPE(lss, LSS_QUERY_MAX_SPEED_RPM, 0, queryType);
	return (int8_t)generic_read_s16(lss, LSS_QUERY_MAX_SPEED_RPM);
}

LSS_LED_Color get_color_led(LSS* lss, LSS_QueryType queryType)
{
	CHECK_COMM_STATUS_TYPE(lss, LSS_QUERY_COLOR_LED, LSS_LED_Black, queryType);
	return (LSS_LED_Color)generic_read_s16(lss, LSS_QUERY_COLOR_LED);
}

LSS_ConfigGyre get_gyre(LSS* lss, LSS_QueryType queryType)
{
	CHECK_COMM_STATUS_TYPE(lss, LSS_QUERY_GYRE, (LSS_ConfigGyre)0, queryType);
	return (LSS_ConfigGyre) generic_read_s16(lss, LSS_QUERY_GYRE);
}


uint16_t get_voltage(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_VOLTAGE, 0);
	return generic_read_s16(lss, LSS_QUERY_VOLTAGE);
}

uint16_t get_temperature(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_TEMPERATURE, 0);
	return generic_read_s16(lss, LSS_QUERY_TEMPERATURE);
}

uint16_t get_current(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_CURRENT, 0);
	return generic_read_s16(lss, LSS_QUERY_CURRENT);
}

uint16_t get_analog(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_ANALOG, 0);
	return generic_read_s16(lss, LSS_QUERY_ANALOG);
}

uint16_t get_distance_mm(LSS* lss, LSS_QueryTypeDistance queryTypeDistance)
{
	CHECK_COMM_STATUS_TYPE(lss, LSS_QUERY_MODEL_STRING, 0, queryTypeDistance);
	return generic_read_s16(lss, LSS_QUERY_ANALOG);
}

LSS_Model get_model(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_MODEL_STRING, LSS_ModelUnknown);

	char* valueStr = generic_read_str(lss, LSS_QUERY_MODEL_STRING);

	if (strcmp(valueStr, LSS_MODEL_HT1) == 0)
	{
		return LSS_ModelHighTorque;
	}
	else if (strcmp(valueStr, LSS_MODEL_ST1) == 0)
	{
		return LSS_ModelStandard;
	}
	else if (strcmp(valueStr, LSS_MODEL_HS1) == 0)
	{
		return LSS_ModelHighSpeed;
	}
	else
	{
		return LSS_ModelUnknown;
	}
}

char* get_serial_number(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_SERIAL_NUMBER, NULL);
	return generic_read_str(lss, LSS_QUERY_SERIAL_NUMBER);
}

uint16_t get_firmware_version(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_FIRMWARE_VERSION, 0);
	return generic_read_s16(lss, LSS_QUERY_FIRMWARE_VERSION);
}

int8_t get_angular_stiffness(LSS* lss, LSS_QueryType queryType)
{
	CHECK_COMM_STATUS_TYPE(lss, LSS_QUERY_ANGULAR_STIFFNESS, 0, queryType);
	return (int8_t)generic_read_s16(lss, LSS_QUERY_ANGULAR_STIFFNESS);
}

int8_t get_angular_holding_stiffness(LSS* lss, LSS_QueryType queryType)
{
	CHECK_COMM_STATUS_TYPE(lss, LSS_QUERY_ANGULAR_HOLDING_STIFFNESS, 0, queryType);
	return (int8_t)generic_read_s16(lss, LSS_QUERY_ANGULAR_HOLDING_STIFFNESS);
}

int16_t get_angular_acceleration(LSS* lss, LSS_QueryType queryType)
{
	CHECK_COMM_STATUS_TYPE(lss, LSS_QUERY_ANGULAR_ACCELERATION, 0, queryType);
	return (int16_t)generic_read_s16(lss, LSS_QUERY_ANGULAR_ACCELERATION);
}

int16_t get_angular_deceleration(LSS* lss, LSS_QueryType queryType)
{
	CHECK_COMM_STATUS_TYPE(lss, LSS_QUERY_ANGULAR_DECELERATION, 0, queryType);
	return (int16_t)generic_read_s16(lss, LSS_QUERY_ANGULAR_DECELERATION);
}

bool get_is_motion_control_enabled(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_ENABLE_MOTION_CONTROL, false);
	return (bool)generic_read_s16(lss, LSS_QUERY_ENABLE_MOTION_CONTROL);
}

int16_t get_filter_position_count(LSS* lss, LSS_QueryType queryType)
{
	CHECK_COMM_STATUS_TYPE(lss, LSS_QUERY_FILTER_POSITION_COUNT, 0, queryType);
	return (int16_t)generic_read_s16(lss, LSS_QUERY_FILTER_POSITION_COUNT);
}

uint8_t get_blinking_led(LSS* lss)
{
	CHECK_COMM_STATUS(lss, LSS_QUERY_BLINKING_LED, 0);
	return (uint8_t)generic_read_s16(lss, LSS_QUERY_BLINKING_LED);
}


/* ------- */
/* Configs */
bool set_origin_offset(LSS* lss, int16_t value, LSS_SetType setType)
{
	return set_session_config(lss, setType, value, LSS_ACTION_ORIGIN_OFFSET, LSS_CONFIG_ORIGIN_OFFSET);
}

bool set_angular_range(LSS* lss, uint16_t value, LSS_SetType setType)
{
	return set_session_config(lss, setType, value, LSS_ACTION_ANGULAR_RANGE, LSS_CONFIG_ANGULAR_RANGE);
}

bool set_max_speed(LSS* lss, uint16_t value, LSS_SetType setType)
{
	return set_session_config(lss, setType, value, LSS_ACTION_MAX_SPEED, LSS_CONFIG_MAX_SPEED);
}

bool set_max_speed_rpm(LSS* lss, int8_t value, LSS_SetType setType)
{
	return set_session_config(lss, setType, value, LSS_ACTION_MAX_SPEED_RPM, LSS_CONFIG_MAX_SPEED_RPM);
}

bool set_color_led(LSS* lss, LSS_LED_Color value, LSS_SetType setType)
{
	return set_session_config(lss, setType, value, LSS_ACTION_COLOR_LED, LSS_CONFIG_COLOR_LED);
}

bool set_gyre(LSS* lss, LSS_ConfigGyre value, LSS_SetType setType)
{
	return set_session_config(lss, setType, value, LSS_ACTION_GYRE_DIRECTION, LSS_CONFIG_GYRE_DIRECTION);
}

bool set_first_position(LSS* lss,int16_t value)
{
	return generic_write_val (lss, LSS_CONFIG_FIRST_POSITION, value);
}

bool clear_first_position(LSS* lss)
{
	return generic_write(lss, LSS_CONFIG_FIRST_POSITION);
}

bool set_mode(LSS* lss, LSS_ConfigMode value)
{
	return generic_write_val(lss, LSS_CONFIG_MODE_RC, value);
}

bool set_angular_stiffness(LSS* lss, int8_t value, LSS_SetType setType)
{
	return set_session_config(lss, setType, value, LSS_ACTION_ANGULAR_STIFFNESS, LSS_CONFIG_ANGULAR_STIFFNESS);
}

bool set_angular_holding_stiffness(LSS* lss, int8_t value, LSS_SetType setType)
{
	return set_session_config(lss, setType, value, LSS_ACTION_ANGULAR_HOLDING_STIFFNESS, LSS_CONFIG_ANGULAR_HOLDING_STIFFNESS);
}

bool set_angular_acceleration(LSS* lss, int16_t value, LSS_SetType setType)
{
	return set_session_config(lss, setType, value, LSS_ACTION_ANGULAR_ACCELERATION, LSS_CONFIG_ANGULAR_ACCELERATION);
}

bool set_angular_deceleration(LSS* lss, int16_t value, LSS_SetType setType)
{
	return set_session_config(lss, setType, value, LSS_ACTION_ANGULAR_DECELERATION, LSS_CONFIG_ANGULAR_DECELERATION);
}

bool set_motion_control_enabled(LSS* lss, bool value)
{
	return generic_write_val(lss, LSS_ACTION_ENABLE_MOTION_CONTROL, value);
}

bool set_filter_position_count(LSS* lss, int16_t value, LSS_SetType setType)
{
	return set_session_config(lss, setType, value, LSS_FILTER_POSITION_COUNT, LSS_CONFIG_FILTER_POSITION_CURRENT);
}

bool set_blinking_led(LSS* lss, uint8_t value)
{
	return generic_write_val(lss, LSS_CONFIG_BLINKING_LED, value);
}


/*************************************************************************************************/
/* Private functions definitions --------------------------------------------------------------- */

static bool set_session_config (LSS* lss, LSS_SetType setType, int16_t value,
								const char* sessionAction, const char* configAction)
{
	if (setType == LSS_SetSession)
	{
		return generic_write_val(lss, sessionAction, value);
	}
	else if (setType == LSS_SetConfig)
	{
		return generic_write_val(lss, configAction, value);
	}
	else
	{
		return false;
	}
}

static int16_t timed_read(LSS* lss)
{
	-----
	int c;
	unsigned long startMillis = millis();
	do
	{
		c =  bus->read();
		if (c >= 0)
			return (c);
	} while (millis() - startMillis < _msg_char_timeout);
	return (-1);     // -1 indicates timeout
}


static void set_read_timeouts(LSS* lss, uint32_t startResponseTimeout, uint32_t msgCharTimeout)
{
	lss->msgCharTimeout = msgCharTimeout;

	-----
	bus->setTimeout(start_response_timeout);

}

static void init_bus(LSS* lss, UART_HandleTypeDef* huart, uint32_t baud)
{
	-----
	bus = &s;
	bus->setTimeout(LSS_TIMEOUT);
	hardwareSerial = true;
	s.begin(baud);
}

// Close the bus (stream), free pins and null reference
static void close_bus(LSS* lss)
{
	-----
}


/* ------- */
/* Writing */

/* Build & write a LSS command to the bus using the provided ID (no value)
 * Max size for cmd = (LSS_MAX_TOTAL_COMMAND_LENGTH - 1) */
static bool generic_write(LSS* lss, const char* cmd)
{
	-----
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (false);
	}

	// Build command
	bus->write('#');
	// Servo ID
	bus->print(id, DEC);
	// Command
	bus->write(cmd);
	// Command end
	bus->write('\r');
	// Success
	lastCommStatus = LSS_CommStatus_WriteSuccess;
	return (true);
}

/* Build & write a LSS command to the bus using the provided ID and value
 * Max size for cmd = (LSS_MAX_TOTAL_COMMAND_LENGTH - 1) */
static bool generic_write_val(LSS* lss, const char* cmd, int16_t value)
{
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (false);
	}

	bus->write('#');
	// Servo ID
	bus->print(id, DEC);
	// Command
	bus->write(cmd);
	// Value
	bus->print(value, DEC);
	// Command end
	bus->write('\r');
	// Success
	lastCommStatus = LSS_CommStatus_WriteSuccess;
	return (true);
}

// Build & write a LSS command to the bus using the provided ID and value
// Max size for cmd = (LSS_MAX_TOTAL_COMMAND_LENGTH - 1)
static bool generic_write_val_param(LSS* lss, const char* cmd, int16_t value,
                                    const char* parameter, int16_t parameter_value)
{
	-----
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (false);
	}

	bus->write('#');
	// Servo ID
	bus->print(id, DEC);
	// Command
	bus->print(cmd);
	// Value
	bus->print(value, DEC);
	bus->write(parameter);
	// Parameter Value
	bus->print(parameter_value, DEC);
	// Command end
	bus->write('\r');
	// Success
	lastCommStatus = LSS_CommStatus_WriteSuccess;
	return (true);
}


/* ------- */
/* Reading */

static char* generic_read_str(LSS* lss, const char* cmd)
{
	-----
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = LSS_CommStatus_ReadNoBus;
		return ((char *) nullptr);
	}

	// 

	// Read from bus until first character; exit if not found before timeout
	if (!(bus->find(LSS_COMMAND_REPLY_START)))
	{
		lastCommStatus = LSS_CommStatus_ReadTimeout;
		return ((char *) nullptr);
	}

	// Ok we have the * now now lets get the servo ID from the message.
	readID = 0;
	int c;
	bool valid_field = 0;
	while ((c =  timedRead()) >= 0)
	{
		if ((c < '0') || (c > '9')) break;	// not a number character
		readID = readID * 10 + c - '0';
		valid_field = true;
	}

	if ((!valid_field) || (readID != id))
	{
		lastCommStatus = LSS_CommStatus_ReadWrongID;
		// BUGBUG Should we clear out until CR?
		return ((char *) nullptr);
	} 

	// Now lets validate the right CMD
	for (;;)
	{
		if (c != *cmd)
		{
			lastCommStatus = LSS_CommStatus_ReadWrongIdentifier;
			return ((char *) nullptr);			
		}
		cmd++;
		if (*cmd == '\0')
			break;
		c =  timedRead();
	}
	size_t maxLength = (LSS_MAX_TOTAL_COMMAND_LENGTH - 1);
	size_t index = 0;


	while (index < maxLength)
	{
		c =  timedRead();
		if (c < 0 || c == LSS_COMMAND_END[0])
			break;
		value[index] = (char) c;
		index++;
	}
	value[index] = '\0';

	// Return value (success)
	if (c < 0) 
	{
		// did not get the ending CR
		lastCommStatus = LSS_CommStatus_ReadTimeout;
		return ((char *) nullptr);
	}

	lastCommStatus = LSS_CommStatus_ReadSuccess;
	return value;
}


static uint16_t generic_read_s16(LSS* lss, const char* cmd)
{
	-----
	// Let the string function do all of the main parsing work.
	char *valueStr = genericRead_Blocking_str(id, cmd);
	if (valueStr == (char *) nullptr)
	{
		// the above method will have already set the error condition. 
		return 0;
	}

	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = LSS_CommStatus_ReadNoBus;
		return (0);
	}

	// convert the value string to value
	int16_t value = 0;
	int16_t value_sign = 1;
	for(;;)
	{
		if ((*valueStr >= '0') && (*valueStr <= '9'))
			value = value * 10 + *valueStr - '0';
		else if (*valueStr == '-')
			value_sign = -1;
		else 
			break;
		valueStr++; 
	}
	// now see if we exited with valid number
	if (*valueStr != '\0')
	{
		lastCommStatus = LSS_CommStatus_ReadWrongID;
		return (0);
	}
	// return the computed value
	return value * value_sign; 
} 
