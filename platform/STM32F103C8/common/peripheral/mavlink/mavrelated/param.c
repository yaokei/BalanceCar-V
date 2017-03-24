
#include "stdlib.h"
#include "string.h"
#include "param.h"

struct param_s _param[] = {
	/*****************************************************************
	 *  velocity
	 ****************************************************************/
  {
		"VEL_TARGET",
		PARAM_TYPE_FLOAT,
		.val.f = 0.0f
	},
	
	{
		"VEL_KP",
		PARAM_TYPE_FLOAT,
		.val.f = 0.95f
	},
	
	{
		"VEL_KI",
		PARAM_TYPE_FLOAT,
		.val.f = 0.22f
	},
	
	{
		"VEL_KD",
		PARAM_TYPE_FLOAT,
		.val.f = 0.0f
	},
	
	/*****************************************************************
	 *  balance
	 ****************************************************************/	
	{
		"BAL_KP",
		PARAM_TYPE_FLOAT,
		.val.f = 20.0f
	},
	
	{
		"BAL_KI",
		PARAM_TYPE_FLOAT,
		.val.f = 0.0f
	},
	
	{
		"BAL_KD",
		PARAM_TYPE_FLOAT,
		.val.f = 1.0f
	},
	
	/*****************************************************************
	 *  sensor
	 ****************************************************************/
	{
		"ACC_OFFSET_X",
		PARAM_TYPE_FLOAT,
		.val.f = -450.0f
	},
	
	{
		"ACC_OFFSET_Y",
		PARAM_TYPE_FLOAT,
		.val.f = 10.0f
	},
	
	{
		"ACC_OFFSET_Z",
		PARAM_TYPE_FLOAT,
		.val.f = 150.0f
	},
	
	{
		"GRYO_OFFSET_X",
		PARAM_TYPE_FLOAT,
		.val.f = 10.0f
	},
	
	{
		"GRYO_OFFSET_Y",
		PARAM_TYPE_FLOAT,
		.val.f = -8.0f
	},
	
	{
		"GRYO_OFFSET_Z",
		PARAM_TYPE_FLOAT,
		.val.f = 7.0f
	},
	
//	{
//		"GY_RATIO",
//		PARAM_TYPE_FLOAT,
//		.val.f = 10.0f
//	},
//	
	
	/*****************************************************************
	 *  others
	 ****************************************************************/
	{
		"LIMIT_FRICTION",
		PARAM_TYPE_FLOAT,
		.val.f = 10.0f
	},
	
	{
		"KALMAN_Q",
		PARAM_TYPE_FLOAT,
		.val.f = 0.2f
	},
	
	{
		"KALMAN_R",
		PARAM_TYPE_FLOAT,
		.val.f = 0.1f
	},
	/*****************************************************************
	 * MAVLink
	 ****************************************************************/

	{
		"MAV_SYS_ID",
		PARAM_TYPE_INT32,
		.val.i = 1
	},

	{
		"MAV_COMP_ID",
		PARAM_TYPE_INT32,
		.val.i = 1
	},

	{
		"MAV_PROTO_VER",
		PARAM_TYPE_INT32,
		.val.i = 1
	},

	{
		"MAV_RADIO_ID",
		PARAM_TYPE_INT32,
		.val.i = 0
	},

	{
		"MAV_TYPE",
		PARAM_TYPE_INT32,
		.val.i = 2
	},

	{
		"MAV_USEHILGPS",
		PARAM_TYPE_INT32,
		.val.i = 0
	},

	{
		"MAV_FWDEXTSP",
		PARAM_TYPE_INT32,
		.val.i = 1
	},

	{
		"MAV_BROADCAST",
		PARAM_TYPE_INT32,
		.val.i = 0
	},

	{
		"MAV_TEST_PAR",
		PARAM_TYPE_INT32,
		.val.i = 1
	}
};

static unsigned count = sizeof(_param) / sizeof(struct param_s);

/**
 * Return the total number of parameters.
 *
 * @return  The number of parameters.
 */
unsigned param_count(void)
{
    return count;
}

/**
 * Look up a parameter by name.
 *
 * @param name  The canonical name of the parameter being looked up.
 * @return      A handle to the parameter, or PARAM_INVALID if the parameter does not exist.
 *          This call will also set the parameter as "used" in the system, which is used
 *          to e.g. show the parameter via the RC interface
 */
unsigned param_find(const char *name)
{
	for (uint32_t i = 0; i < count; i++) {
		if (!strcmp(_param[i].name, name)) {
			return i;
		}
	}

	return PARAM_INVALID;
}

/**
 * Obtain the name of a parameter.
 *
 * @param param     A handle returned by param_find or passed by param_foreach.
 * @return          The name assigned to the parameter, or NULL if the handle is invalid.
 */
const char *param_name(uint32_t param)
{
    if (param < count) {
        return _param[param].name;
    } else {
        return NULL;
    }
}

/**
 * Obtain the type of a parameter.
 *
 * @param param     A handle returned by param_find or passed by param_foreach.
 * @return          The type assigned to the parameter.
 */
param_type_t param_type(uint32_t param)
{
    if (param < count) {
        return _param[param].type;
    } else {
        return PARAM_TYPE_UNKNOWN;
    }
}

/**
 * Determine the size of a parameter.
 *
 * @param param     A handle returned by param_find or passed by param_foreach.
 * @return          The size of the parameter's value.
 */
unsigned param_size(uint32_t param)
{
    if (param < count) {
        return sizeof(_param[param]);
    } else {
        return NULL;
    }
}

/**
 * Copy the value of a parameter.
 *
 * @param param     A handle returned by param_find or passed by param_foreach.
 * @param val       Where to return the value, assumed to point to suitable storage for the parameter type.
 *              For structures, a bitwise copy of the structure is performed to this address.
 * @return          Zero if the parameter's value could be returned, nonzero otherwise.
 */
int param_get(uint32_t param, void *val)
{
    if (param < count) {
        memcpy(val, &_param[param].val, sizeof(_param[param].val));
        return 0;
    } else {
        return -1;
    }
}

/**
 * Set the value of a parameter.
 *
 * @param param     A handle returned by param_find or passed by param_foreach.
 * @param val       The value to set; assumed to point to a variable of the parameter type.
 *              For structures, the pointer is assumed to point to a structure to be copied.
 * @return          Zero if the parameter's value could be set from a scalar, nonzero otherwise.
 */

static int check = 1;

int param_set(uint32_t param, const void *val)
{
    if (param < count) {
        memcpy(&_param[param].val, val, sizeof(_param[param].val));
        check++;
        return 0;
    } else {
        return -1;
    }
}

int param_check(void)
{
    return check;
}
