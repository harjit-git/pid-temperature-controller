#include <msp430.h>

/* =========================================================
   EP3BB3 Final Project - MSP430G2553
   PID Temperature Controller

   UART Protocol:
   - MATLAB sends setpoint:  "25.0\n"
   - MATLAB sends PID gains: "K:30.00,0.08,2.00\n" 
   - MSP430 sends CSV:       "temp,setpoint,control,pwm\r\n"
   - MSP430 sends debug:     "PWM%:xx IN1:x IN2:x MODE:XXXX\r\n"

   Pin map:
   - P1.0 = Thermistor (A0)
   - P1.6 = PWM -> L298N ENA
   - P2.0 = L298N IN1
   - P2.1 = L298N IN2
   - P1.1 = UART RX
   - P1.2 = UART TX

   ========================================================= */

#define VCC             3.3f
#define ADC_MAX         1023.0f
#define R_FIXED         10000.0f
#define PWM_PERIOD      1000U       /* 1 kHz at 1 MHz SMCLK */
#define SAMPLE_MS       200U        /* Control loop period   */
#define ADC_SAMPLES     16          /* Average this many ADC reads */

#define TEC_IN1_BIT     BIT0        /* P2.0 */
#define TEC_IN2_BIT     BIT1        /* P2.1 */

#define RX_BUF_LEN      32
#define TABLE_SIZE      34

/* ------ Thermistor lookup (muRata NCP21XV103J03RA) ------ */
static const float temp_table[TABLE_SIZE] = {
    -40,-35,-30,-25,-20,-15,-10, -5,
      0,  5, 10, 15, 20, 25, 30, 35,
     40, 45, 50, 55, 60, 65, 70, 75,
     80, 85, 90, 95,100,105,110,115,
    120,125
};
static const float res_table[TABLE_SIZE] = {
    328996.0f,237387.0f,173185.0f,127773.0f,
     95327.0f, 71746.0f, 54564.0f, 41813.0f,
     32330.0f, 25194.0f, 19785.0f, 15651.0f,
     12468.0f, 10000.0f,  8072.0f,  6556.0f,
      5356.0f,  4401.0f,  3635.0f,  3019.0f,
      2521.0f,  2115.0f,  1781.0f,  1509.0f,
      1284.0f,  1097.0f,   941.0f,   810.0f,
       701.0f,   608.0f,   530.0f,   463.0f,
       406.0f,   358.0f
};

/* ------ Globals ------ */
volatile unsigned int  ms_counter  = 0;
volatile unsigned char sample_flag = 0;

volatile char          rx_buf[RX_BUF_LEN];
volatile unsigned char rx_idx  = 0;
volatile unsigned char rx_ready = 0;

/* PID state */
float setpoint_c     = 25.0f;
float current_temp_c = 25.0f;
float control_output = 0.0f;

/* PID gains - tunable via MATLAB GUI (BONUS) */
float Kp = 30.0f;
float Ki = 0.08f;
float Kd = 2.0f;

float pid_integral   = 0.0f;
float pid_prev_error = 0.0f;

/* ------ Prototypes ------ */
void clock_init(void);
void uart_init(void);
void adc_init(void);
void pwm_init(void);
void timer1_init(void);

void uart_putc(char c);
void uart_puts(const char *s);
void uart_put_uint(unsigned int n);
void uart_put_fixed1(float x);
void uart_print_debug(void);

float parse_float_simple(const char *s);
void  parse_pid_command(const char *s);   /* K:kp,ki,kd */

unsigned int  adc_read_raw(void);
float         interpolate_temp(float R);
float         readTemperature(void);

float pid_update(float sp, float pv, float dt);
void  tec_set_output(float u);

/* ========================================================= */
int main(void)
{
    char local_buf[RX_BUF_LEN];
    unsigned char i;
    float new_sp;

    WDTCTL = WDTPW | WDTHOLD;

    clock_init();
    uart_init();
    adc_init();
    pwm_init();
    timer1_init();

    __bis_SR_register(GIE);

    /* Header - MATLAB skips this line */
    uart_puts("=== PID Temperature Controller ===\r\n");
    uart_puts("temp,setpoint,control,pwm,Kp,Ki,Kd\r\n");

    while (1)
    {
        /* ---- Parse incoming UART commands ---- */
        if (rx_ready)
        {
            __bic_SR_register(GIE);
            for (i = 0; i < RX_BUF_LEN; i++)
                local_buf[i] = rx_buf[i];
            rx_ready = 0;
            __bis_SR_register(GIE);

            /*  PID gains from MATLAB: "K:30.00,0.08,2.00" */
            if (local_buf[0] == 'K' && local_buf[1] == ':') {
                parse_pid_command(&local_buf[2]);
                pid_integral   = 0.0f;   /* reset integral on gain change */
                pid_prev_error = 0.0f;
                uart_puts(">> PID updated Kp=");
                uart_put_fixed1(Kp);
                uart_puts(" Ki=");
                uart_put_fixed1(Ki);
                uart_puts(" Kd=");
                uart_put_fixed1(Kd);
                uart_puts("\r\n");
            }
            else {
                /* Normal setpoint: "25.0" */
                new_sp = parse_float_simple(local_buf);
                if (new_sp >= 5.0f && new_sp <= 45.0f) {
                    setpoint_c     = new_sp;
                    pid_integral   = 0.0f; /* reset on setpoint change */
                    pid_prev_error = 0.0f;
                    uart_puts(">> Setpoint: ");
                    uart_put_fixed1(setpoint_c);
                    uart_puts(" C\r\n");
                }
            }
        }

        /* ---- 200 ms control loop ---- */
        if (sample_flag)
        {
            sample_flag = 0;

            current_temp_c = readTemperature();
            control_output = pid_update(setpoint_c, current_temp_c, 0.2f);
            tec_set_output(control_output);

            /* CSV line: temp,setpoint,control,pwm% */
            uart_put_fixed1(current_temp_c);  uart_putc(',');
            uart_put_fixed1(setpoint_c);      uart_putc(',');
            uart_put_fixed1(control_output);  uart_putc(',');
            uart_put_fixed1(((float)TA0CCR1 / (float)(PWM_PERIOD-1)) * 100.0f); uart_putc(',');
            uart_put_fixed1(Kp);              uart_putc(',');
            uart_put_fixed1(Ki);              uart_putc(',');
            uart_put_fixed1(Kd);
            uart_puts("\r\n");

            /* Debug line for PuTTY */
            uart_print_debug();
        }

        __no_operation();
    }
}

/* ----------------------------------------------------------
   Parse "30.00,0.08,2.00" and update Kp, Ki, Kd
   ---------------------------------------------------------- */
void parse_pid_command(const char *s)
{
    unsigned int i = 0;
    float vals[3] = {Kp, Ki, Kd};
    unsigned char idx = 0;

    while (idx < 3)
    {
        vals[idx] = parse_float_simple(&s[i]);
        /* advance past current number to next comma */
        while (s[i] != '\0' && s[i] != ',') i++;
        if (s[i] == ',') i++;
        idx++;
    }
    Kp = vals[0];
    Ki = vals[1];
    Kd = vals[2];
}

/* ----------------------------------------------------------
   H-bridge debug line
   ---------------------------------------------------------- */
void uart_print_debug(void)
{
    float duty_pct;
    unsigned char in1, in2;

    in1      = (P2OUT & TEC_IN1_BIT) ? 1 : 0;
    in2      = (P2OUT & TEC_IN2_BIT) ? 1 : 0;
    duty_pct = ((float)TA0CCR1 / (float)( _PERIOD - 1)) * 100.0f;

    uart_puts("PWM%:"); uart_put_fixed1(duty_pct);
    uart_puts(" IN1:"); uart_putc(in1 ? '1' : '0');
    uart_puts(" IN2:"); uart_putc(in2 ? '1' : '0');
    uart_puts(" MODE:");
    if      (TA0CCR1 == 0)    uart_puts("OFF");
    else if (in1 && !in2)     uart_puts("HEAT");
    else if (!in1 && in2)     uart_puts("COOL");
    else                      uart_puts("ERR");
    uart_puts("\r\n");
}

/* ---------- Clock: 1 MHz DCO ---------- */
void clock_init(void)
{
    if (CALBC1_1MHZ == 0xFF) { while (1); } /* erased calibration = halt */
    DCOCTL  = 0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL  = CALDCO_1MHZ;
}

/* ---------- UART: 9600 baud on P1.1/P1.2 ---------- */
void uart_init(void)
{
    P1SEL  |= BIT1 | BIT2;
    P1SEL2 |= BIT1 | BIT2;
    UCA0CTL1 |= UCSWRST;
    UCA0CTL1 |= UCSSEL_2;   /* SMCLK */
    UCA0BR0   = 104;         /* 1 MHz / 9600 */
    UCA0BR1   = 0;
    UCA0MCTL  = UCBRS0;
    UCA0CTL1 &= ~UCSWRST;
    IE2 |= UCA0RXIE;         /* Enable RX interrupt */
}

/* ---------- ADC10 on A0 / P1.0 ---------- */
void adc_init(void)
{
    ADC10CTL0 = SREF_0 | ADC10SHT_3 | ADC10ON; /* Vcc ref, 64-cycle SH */
    ADC10CTL1 = INCH_0;
    ADC10AE0 |= BIT0;
}

/* ---------- PWM on P1.6 / TA0.1 (1 kHz) ---------- */
void pwm_init(void)
{
    P1DIR  |=  BIT6;
    P1SEL  |=  BIT6;
    P1SEL2 &= ~BIT6;

    P2DIR |= TEC_IN1_BIT | TEC_IN2_BIT;
    P2OUT &= ~(TEC_IN1_BIT | TEC_IN2_BIT);

    TA0CCR0  = PWM_PERIOD - 1;
    TA0CCTL1 = OUTMOD_7;        /* Reset/Set mode */
    TA0CCR1  = 0;
    TA0CTL   = TASSEL_2 | MC_1 | TACLR;
}

/* ---------- Timer1_A: 1 ms interrupt tick ---------- */
void timer1_init(void)
{
    TA1CCR0  = 1000 - 1;    /* 1 ms at 1 MHz */
    TA1CCTL0 = CCIE;
    TA1CTL   = TASSEL_2 | MC_1 | TACLR;
}

/* ---------- UART helpers ---------- */
void uart_putc(char c)
{
    while (!(IFG2 & UCA0TXIFG));
    UCA0TXBUF = c;
}
void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

void uart_put_uint(unsigned int n)
{
    char buf[6];
    unsigned char i = 0;
    if (n == 0) { uart_putc('0'); return; }
    while (n > 0 && i < sizeof(buf)) { buf[i++] = '0' + (n % 10); n /= 10; }
    while (i > 0) uart_putc(buf[--i]);
}

void uart_put_fixed1(float x)
{
    long x10;
    unsigned int whole, frac;
    if (x < 0) { uart_putc('-'); x = -x; }
    x10   = (long)(x * 10.0f + 0.5f);
    whole = (unsigned int)(x10 / 10);
    frac  = (unsigned int)(x10 % 10);
    uart_put_uint(whole);
    uart_putc('.');
    uart_putc('0' + frac);
}

/* ---------- Simple float parser ---------- */
float parse_float_simple(const char *s)
{
    int sign = 1;
    long whole = 0, frac = 0, frac_div = 1;
    unsigned int i = 0;

    if      (s[0] == '-') { sign = -1; i++; }
    else if (s[0] == '+') { i++; }

    while (s[i] >= '0' && s[i] <= '9') { whole = whole*10 + (s[i]-'0'); i++; }
    if (s[i] == '.') {
        i++;
        while (s[i] >= '0' && s[i] <= '9') {
            frac = frac*10 + (s[i]-'0');
            frac_div *= 10;
            i++;
        }
    }
    return sign * ((float)whole + (float)frac/(float)frac_div);
}

/* ---------- ADC read (single sample) ---------- */
unsigned int adc_read_raw(void)
{
    ADC10CTL0 &= ~ENC;
    while (ADC10CTL1 & ADC10BUSY);
    ADC10CTL1  = INCH_0;
    ADC10CTL0 |= ENC | ADC10SC;
    while (ADC10CTL1 & ADC10BUSY);
    return ADC10MEM;
}

/* ---------- Thermistor table interpolation ---------- */
float interpolate_temp(float R)
{
    int i;
    float t;
    for (i = 0; i < TABLE_SIZE - 1; i++) {
        if (R <= res_table[i] && R >= res_table[i+1]) {
            t = (R - res_table[i]) / (res_table[i+1] - res_table[i]);
            return temp_table[i] + t * (temp_table[i+1] - temp_table[i]);
        }
    }
    if (R > res_table[0])          return temp_table[0];
    return temp_table[TABLE_SIZE-1];
}

/* ---------- ADC -> Temperature (16-sample average) ---------- */
/* Circuit: VCC -- R_FIXED -- [ADC node] -- Rtherm -- GND
   Vadc = VCC * Rtherm / (R_FIXED + Rtherm)
   Rtherm = R_FIXED * Vadc / (VCC - Vadc)               */
float readTemperature(void)
{
    unsigned int i;
    unsigned long sum = 0;
    unsigned int  raw; 
    float Vadc, R_therm;

    for (i = 0; i < ADC_SAMPLES; i++)
        sum += adc_read_raw();

    raw   = (unsigned int)(sum / ADC_SAMPLES);
    Vadc  = ((float)raw / ADC_MAX) * VCC;

    /* Clamp to avoid divide-by-zero */
    if (Vadc <= 0.001f)          Vadc = 0.001f;
    if (Vadc >= (VCC - 0.001f)) Vadc = VCC - 0.001f;

    R_therm = R_FIXED * Vadc / (VCC - Vadc);
    return interpolate_temp(R_therm);
}

/* ---------- PID controller ---------- */
/* Output range: -100 (full cool) to +100 (full heat)
   dt = 0.2 s (200 ms sample period)                   */
float pid_update(float sp, float pv, float dt)
{
    float error      = sp - pv;
    float derivative = (error - pid_prev_error) / dt;
    float u;

    pid_integral += error * dt;

    /* Anti-windup clamp */
    if (pid_integral >  100.0f) pid_integral =  100.0f;
    if (pid_integral < -100.0f) pid_integral = -100.0f;

    u = Kp*error + Ki*pid_integral + Kd*derivative;

    pid_prev_error = error;

    if (u >  100.0f) u =  100.0f;
    if (u < -100.0f) u = -100.0f;
    return u;
}

/* ---------- TEC output via L298N H-bridge ---------- */
/* u > 0 -> heat (IN1=1,IN2=0), u < 0 -> cool (IN1=0,IN2=1)
   |u| maps linearly to PWM duty 0-99.9%               */
void tec_set_output(float u)
{
    float mag  = (u < 0.0f) ? -u : u;
    unsigned int duty;

    if (mag > 100.0f) mag = 100.0f;
    duty    = (unsigned int)((mag / 100.0f) * (float)(PWM_PERIOD - 1));
    TA0CCR1 = duty;

    if (duty == 0) {
        P2OUT &= ~(TEC_IN1_BIT | TEC_IN2_BIT);  /* OFF  */
    } else if (u > 0.0f) {
        P2OUT |=  TEC_IN1_BIT;                   /* HEAT */
        P2OUT &= ~TEC_IN2_BIT;
    } else {
        P2OUT &= ~TEC_IN1_BIT;                   /* COOL */
        P2OUT |=  TEC_IN2_BIT;
    }
}

/* ========== ISR: Timer1_A0 - 1 ms tick ========== */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void)
{
    ms_counter++;
    if (ms_counter >= SAMPLE_MS) {
        ms_counter = 0;
        sample_flag = 1;
    }
}

/* ========== ISR: UART RX ========== */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    char c;
    if (IFG2 & UCA0RXIFG) {
        c = UCA0RXBUF;
        if (c == '\r') return;
        if (c == '\n') {
            rx_buf[rx_idx] = '\0';
            rx_idx   = 0;
            rx_ready = 1;
        } else {
            if (rx_idx < (RX_BUF_LEN - 1))
                rx_buf[rx_idx++] = c;
            else
                rx_idx = 0;
        }
    }
}