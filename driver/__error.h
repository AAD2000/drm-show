#include "xrt_error_code.h"


struct __err_record {
	xrtErrorCode	zer_err_code;	/* XRT error code */
	u64		zer_ts;		/* timestamp */
};

struct __error {
	int		ze_num;		/* number of errors recorded */
	int		ze_cap;		/* capacity of current error array */
	struct __err_record *ze_err;	/* error array pointer */
};

