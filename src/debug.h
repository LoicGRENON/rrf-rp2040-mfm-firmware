#ifdef DEBUG
#define log_debug(fmt, ...) _log_debug(fmt, __BASE_FILE__, __LINE__ __VA_OPT__(,) __VA_ARGS__)
void _log_debug(const char*, const char* file, uint32_t line, ...);
#else
#define log_debug(fmt, ...) 
#endif