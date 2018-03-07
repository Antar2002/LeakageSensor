#ifndef F_CPU
	#error "F_CPU not defined. Must be defined for set timings in 1-wire module."
#endif

#if F_CPU==10000000
	#define	WAIT_FOR_RESET_END	160
	#define	RESET_SIG_MIN		147
	#define	RESET_SIG_MAX		153
	#define	MAX_RC_BIT_LENGTH	27
	#define	MAX_TX_BIT_LENGTH	70
#endif

#if F_CPU==8000000
	#define	WAIT_FOR_RESET_END	128
	#define	RESET_SIG_MIN		117
	#define	RESET_SIG_MAX		123
	#define	MAX_RC_BIT_LENGTH	22
	#define	MAX_TX_BIT_LENGTH	56
#endif

// Не работает
#if F_CPU==1000000	// 3,4782608 - соотношение тиков таймера на 10 и на 1 МГц
	#define	WAIT_FOR_RESET_END	46
	#define	RESET_SIG_MIN		42
	#define	RESET_SIG_MAX		44
	#define	MAX_RC_BIT_LENGTH	8
	#define	MAX_TX_BIT_LENGTH	20
#endif
