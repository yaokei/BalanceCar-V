
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef int param_t;

// Parameter types.
typedef enum uint32_type_e {
	// globally-known parameter types
	PARAM_TYPE_INT32 = 0,
	PARAM_TYPE_FLOAT,

	// structure parameters; size is encoded in the type value
	PARAM_TYPE_STRUCT = 100,
	PARAM_TYPE_STRUCT_MAX = 16384 +PARAM_TYPE_STRUCT,

	PARAM_TYPE_UNKNOWN = 0xFFFF
} param_type_t;

// Handle returned when a parameter cannot be found.
#define PARAM_INVALID ((uintptr_t)0xFFFFFFFF)

// Magic handle for hash check param
#define PARAM_HASH ((uintptr_t)INT32_MAX)

#ifdef __cplusplus
extern "C"
{
#endif

unsigned param_find(const char *name);
unsigned param_count(void);
const char *param_name(uint32_t param);
param_type_t param_type(uint32_t param);
unsigned param_size(uint32_t param);
int param_get(uint32_t param, void *val);
int param_set(uint32_t param, const void *val);
int param_reset(uint32_t param);
void param_reset_all(void);

void param_change(void);
int param_check(void);

union param_value_u {
    void *p;
	int32_t i;
	float f;
};

struct param_s {
	char *name;
	param_type_t type;
	union param_value_u val;
};

extern struct param_s _param[];

#ifdef __cplusplus
}
#endif
