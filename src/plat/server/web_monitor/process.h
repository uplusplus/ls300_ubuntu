//typedef struct command_context{
//	char *ret_buf;
//	int log_printf(char *buf, int code, char *fmt, ...);
//}cmdctx;

int on_command(const char *cmd,char *ret_buf,int *ctx,int log_printf(char *buf,int *ctx, int code, char *fmt, ...));
