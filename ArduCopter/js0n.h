// key = string to match or null
// klen = key length (or 0), or if null key then len is the array offset value
// json = json object or array
// jlen = length of json
// vlen = where to store return value length
// returns pointer to value and sets len to value length, or 0 if not found
// any parse error will set vlen to the position of the error
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef enum
{
  ENM_STREAM_NONE = 0,
  ENM_STREAM_START,
  ENM_STREAM_PAUSE,
  ENM_STREAM_STOP
} ENM_STREAM_CMD_T;

typedef enum
{
  ENM_RECORD_NONE = 0,
  ENM_RECORD_START,
  ENM_RECORD_PAUSE,
  ENM_RECORD_STOP
} ENM_RECORD_CMD_T;

typedef enum
{
  ENM_ZOOM_NONE = 0,
  ENM_ZOOM_IN,
  ENM_ZOOM_OUT,
  ENM_ZOOM_HOME
} ENM_ZOOM_CMD_T;

#define JSON_STREAM_CMD       "Stream_cmd"
#define JSON_RECORD_CMD       "Record_cmd"
#define JSON_ZOOM_CMD         "Zoom_cmd"
#define JSON_ZOOM_VAL         "Zoom_value" 

const char *js0n(const char *key, size_t klen,
				 const char *json, size_t jlen, size_t *vlen);
#ifdef __cplusplus
} /* extern "C" */
#endif
