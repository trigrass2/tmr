#ifndef __LOG_H__
#define __LOG_H__
#define RETAILMSG(a,b) (void)0
	
#define MPL_LOG(priority, tag, fmt, ...) do { } while (0)
#define MPL_LOGE(fmt, ...) MPL_LOG(LOG_DEBUG, MPL_LOG_TAG, fmt, ##__VA_ARGS__)
#define MPL_LOGD(fmt, ...) MPL_LOG(LOG_DEBUG, MPL_LOG_TAG, fmt, ##__VA_ARGS__)
#define MPL_LOGV(fmt, ...) MPL_LOG(LOG_DEBUG, MPL_LOG_TAG, fmt, ##__VA_ARGS__)
#define MPL_LOGI(fmt, ...) MPL_LOG(LOG_DEBUG, MPL_LOG_TAG, fmt, ##__VA_ARGS__)

static void __print_result_location(int result,
					   const char *file,
					   const char *func, int line)
{
	MPL_LOGE("%s|%s|%d returning %d\n", file, func, line, result);
}

#define LOG_RESULT_LOCATION(condition) \
	do {								\
		__print_result_location((int)(condition), __FILE__,	\
					__func__, __LINE__);		\
	} while (0)

#endif
