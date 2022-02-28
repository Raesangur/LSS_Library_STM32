/**
 *  Author:         Sebastien Parent-Charette (support@robotshop.com)
 *  Version:        1.2.0
 *  Licence:        LGPL-3.0 (GNU Lesser General Public License version 3)
 *  
 *  Description:    A library that makes using the LSS simple.
 *                  Offers support for both Arduino Uno, Mega and others through
 *                  the use of the Stream class for communication.
 *
 *  Modified By:    Pascal-Emmanuel Lachance (raesangur.com)
 *  Modification:   Rewritten the library to work with the STM32's HAL
 */
#ifndef LSS_H
#define LSS_H

/*************************************************************************************************/
/* File includes ------------------------------------------------------------------------------- */
#include <string.h>
#include <stdint.h>


/*************************************************************************************************/
/* Enumss -------------------------------------------------------------------------------------- */
enum LSS_LastCommStatus
{
    LSS_CommStatus_Idle,
    LSS_CommStatus_ReadSuccess,
    LSS_CommStatus_ReadTimeout,
    LSS_CommStatus_ReadWrongID,
    LSS_CommStatus_ReadWrongIdentifier,
    LSS_CommStatus_ReadWrongFormat,
    LSS_CommStatus_ReadNoBus,
    LSS_CommStatus_ReadUnknown,
    LSS_CommStatus_WriteSuccess,
    LSS_CommStatus_WriteNoBus,
    LSS_CommStatus_WriteUnknown
};

enum LSS_Status
{
    LSS_StatusUnknown,
    LSS_StatusLimp,
    LSS_StatusFreeMoving,
    LSS_StatusAccelerating,
    LSS_StatusTravelling,
    LSS_StatusDecelerating,
    LSS_StatusHolding,
    LSS_StatusOutsideLimits,
    LSS_StatusStuck,     // cannot move at current speed setting
    LSS_StatusBlocked,   // same as stuck but reached maximum duty and still can't move
    LSS_StatusSafeMode,
    LSS_StatusLast
};
enum LSS_Model
{
    LSS_ModelHighTorque,
    LSS_ModelStandard,
    LSS_ModelHighSpeed,
    LSS_ModelUnknown
};

//> Parameter for query
enum LSS_QueryType
{
    LSS_QuerySession            = 0,
    LSS_QueryConfig             = 1,
    LSS_QueryInstantaneousSpeed = 2,
    LSS_QueryTargetTravelSpeed  = 3
};

//> Parameter for query distance sensor
enum LSS_QueryTypeDistance
{
    LSS_Query_Sharp_GP2Y0A41SK0F = 1,
    LSS_Query_Sharp_GP2Y0A21YK0F = 2,
    LSS_Query_Sharp_GP2Y0A02YK0F = 3
};

//> Parameter for setter
enum LSS_SetType
{
    LSS_SetSession = 0,
    LSS_SetConfig  = 1
};

//> Parameter for Serial/RC mode change
enum LSS_ConfigMode
{
    LSS_ModeSerial     = 0,
    LSS_ModePositionRC = 1,
    LSS_ModeWheelRC    = 2
};

//> Parameter for gyre direction
enum LSS_ConfigGyre
{
    LSS_GyreInvalid          = 0,
    LSS_GyreClockwise        = 1,
    LSS_GyreCounterClockwise = -1
};

//> LED colors
enum LSS_LED_Color
{
    LSS_LED_Black   = 0,
    LSS_LED_Red     = 1,
    LSS_LED_Green   = 2,
    LSS_LED_Blue    = 3,
    LSS_LED_Yellow  = 4,
    LSS_LED_Cyan    = 5,
    LSS_LED_Magenta = 6,
    LSS_LED_White   = 7
};


/*************************************************************************************************/
/* Inline functions declarattions -------------------------------------------------------------- */
inline static bool    is_AF       (char c);
inline static bool    is_af       (char c);
inline static bool    is_09       (char c);
inline static bool    is_valid_hex(char c);
inline static bool    is_valid_dec(char c);
inline static uint8_t convert_dec (char c);
inline static uint8_t convert_hex (char c);
inline static bool    str_to_int  (char* inputstr, int32_t* intnum);


/*************************************************************************************************/
/* Struct -------------------------------------------------------------------------------------- */

typedef struct {
    uint8_t servoID;
    
    bool                hardwareSerial;
    LSS_LastCommStatus  lastCommStatus;
    uint16_t            readID;
    uint32_t            msgCharTimeout; // timeout waiting for characters inside of packet
    UART_HandleTypeDef* huart;
    
    char values[24];
} LSS;


/*************************************************************************************************/
/* Public functions declarations --------------------------------------------------------------- */

/* ----------- */
/* Constructor */
void LSS_init(LSS* lss, uint8_t id, UART_HandleTypeDef* huart, uint32_t baud);


/* ------- */
/* Actions */
bool reset          (LSS* lss);
bool limp           (LSS* lss);
bool hold           (LSS* lss);
bool move           (LSS* lss, int16_t value);
bool move_t         (LSS* lss, int16_t value, int16_t tValue);
bool move_ch        (LSS* lss, int16_t value, int16_t chValue);
bool move_relative  (LSS* lss, int16_t value);
bool move_relative_t(LSS* lss, int16_t value, int16_t tValue);
bool wheel          (LSS* lss, int16_t value);
bool wheel_rpm      (LSS* lss, int8_t  value);


/* ------- */
/* Queries */
LSS_Status get_status(LSS* lss);

int16_t  get_origin_offset            (LSS* lss, LSS_QueryType queryType);
uint16_t get_angular_range            (LSS* lss, LSS_QueryType queryType);
uint16_t get_position_pulse           (LSS* lss);
int32_t  get_position                 (LSS* lss);
int16_t  get_first_position           (LSS* lss);
bool     get_is_first_position_enabled(LSS* lss);

int16_t  get_speed        (LSS* lss);
int8_t   get_speed_rpm    (LSS* lss);
int8_t   get_speed_pulse  (LSS* lss);
uint16_t get_max_speed    (LSS* lss, LSS_QueryType queryType);
int8_t   get_max_speed_rpm(LSS* lss, LSS_QueryType queryType);

LSS_LED_Color  get_color_led(LSS* lss, LSS_QueryType queryType);
LSS_ConfigGyre get_gyre     (LSS* lss, LSS_QueryType queryType);

uint16_t get_voltage    (LSS* lss);
uint16_t get_temperature(LSS* lss);
uint16_t get_current    (LSS* lss);
uint16_t get_analog     (LSS* lss);
uint16_t get_distance_mm(LSS* lss, LSS_QueryTypeDistance queryTypeDistance);

LSS_Model get_model           (LSS* lss);
char*     get_serial_number   (LSS* lss);
uint16_t  get_firmware_version(LSS* lss);

int8_t  get_angular_stiffness        (LSS* lss, LSS_QueryType queryType);
int8_t  get_angular_holding_stiffness(LSS* lss, LSS_QueryType queryType);
int16_t get_angular_acceleration     (LSS* lss, LSS_QueryType queryType);
int16_t get_angular_deceleration     (LSS* lss, LSS_QueryType queryType);
bool    get_is_motion_control_enabled(LSS* lss);
int16_t get_filter_position_count    (LSS* lss, LSS_QueryType queryType);
uint8_t get_blinking_led             (LSS* lss);


/* ------- */
/* Configs */
bool set_color_led       (LSS* lss, LSS_LED_Color  value, LSS_SetType setType);
bool set_gyre            (LSS* lss, LSS_ConfigGyre value, LSS_SetType setType);
bool set_mode            (LSS* lss, LSS_ConfigMode value);
bool set_origin_offset   (LSS* lss, int16_t  value, LSS_SetType setType);
bool set_angular_range   (LSS* lss, uint16_t value, LSS_SetType setType);
bool set_max_speed       (LSS* lss, uint16_t value, LSS_SetType setType);
bool set_max_speed_rpm   (LSS* lss, int8_t   value, LSS_SetType setType);
bool set_first_position  (LSS* lss, int16_t  value);
bool clear_first_position(LSS* lss);

bool set_angular_stiffness        (LSS* lss, int8_t  value, LSS_SetType setType);
bool set_angular_holding_stiffness(LSS* lss, int8_t  value, LSS_SetType setType);
bool set_angular_acceleration     (LSS* lss, int16_t value, LSS_SetType setType);
bool set_angular_deceleration     (LSS* lss, int16_t value, LSS_SetType setType);
bool set_filter_position_count    (LSS* lss, int16_t value, LSS_SetType setType);
bool set_blinking_led             (LSS* lss, uint8_t value);
bool set_motion_control_enabled   (LSS* lss, bool    value);


/*************************************************************************************************/
/* Inline functions definitions - -------------------------------------------------------------- */
inline static bool is_AF(char c)
{
    return (c >= 'A') && (c <= 'F');
}

inline static bool is_af(char c)
{
    return (c >= 'a') && (c <= 'f');
}

inline static bool is_09(char c)
{
    return (c >= '0') && (c <= '9');
}

inline static bool is_valid_hex(char c)
{
    return is_AF(c) || is_af(c) || is_09(c);
}

inline static bool is_valid_dec(char c)
{
    return is_09(c);    
}

inline static uint8_t convert_dec(char c)
{
    return c - '0';
}

inline static uint8_t convert_hex(char c)
{
    return is_09(c) ? (c - '0')      : 
           is_AF(c) ? (c - 'A' + 10) :
           is_af(c) ? (c - 'a' + 10) :
           0;
}

inline static bool str_to_int(char* inputstr, int32_t* intnum)
{
    const size_t MAX_LENGTH = 11;

    if (inputstr == NULL || inputstr[0] == '\0')
    {
        return false;
    }

    /* Check for hex input */
    if (inputstr[0] == '0' && (inputstr[1] == 'x' || inputstr[1] == 'X'))
    {
        /* "0x" only */
        if (inputstr[2] == '\0')
        {
            *intnum = 0;
            return true;
        }
        else
        {
            int32_t val = 0;
            for (int8_t i = 2; i < MAX_LENGTH; i++)
            {
                /* Check end of string */
                if (inputstr[i] == '\0')
                {
                    return true;
                }

                /* Convert hex char to hex value */
                if (is_valid_hex(inputstr[i]))
                {
                    *intnum = (*intnum << 4) + convert_hex(inputstr[i]);
                }
                else
                {
                    return false;
                }
            }

            /* Over MAX_LENGTH - invalid */
            return false;
        }
    }
    else /* max 10-digit decimal input */
    {
        bool neg = inputstr[0] == '-';  // Check if first character is '-'

        if (neg && inputstr[1] == '\0')
        {
            return false;
        }
        
        for (int i = neg ? 1 : 0; i < MAX_LENGTH; i++)
        {
            if (inputstr[i] == '\0')
            {
                *intnum *= neg ? -1 : 1;    // multiplies by -1 if negative
                return true;
            }

            /* Convert dec char to dec value */
            if (is_valid_dec(inputstr[i]))
            {
                *intnum = *intnum * 10 + convert_dec(inputstr[i]);
            }
            else
            {
                return false;
            }
        }
        /* Over MAX_LENGTH digits decimal - invalid */
        return false;
    }
}


#endif
/*************************************************************************************************/
/* ----- END OF FILE ----- */
