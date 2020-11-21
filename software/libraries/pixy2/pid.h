#define PID_MAX_INTEGRAL         2000
#define ZUMO_BASE_DEADBAND       20

typedef struct PIDLoop {
  int32_t m_command; 

  int32_t m_pgain;
  int32_t m_igain;
  int32_t m_dgain;
  
  int32_t m_prevError;
  int32_t m_integral;
  bool m_servo;
} pid_t;


pid_init(pid_t *p, int32_t pgain, int32_t igain, int32_t dgain, bool servo) {
    p->m_pgain = pgain;
    p->m_igain = igain;
    p->m_dgain = dgain;
    p->m_servo = servo;

    reset(p);
}

void pid_reset(pid_t *p) {
    if (p->m_servo)
        p->m_command = PIXY_RCS_CENTER_POS;
    else
        p->m_command = 0;
        
    p->m_integral = 0;
    p->m_prevError = 0x80000000L;
}  

void pid_update(pid_t *p, int32_t error) {
    int32_t pid;

    if (p->m_prevError != 0x80000000L) { 
        // integrate integral
        p->m_integral += error;
        // bound the integral
        if (p->m_integral > PID_MAX_INTEGRAL)
        p->m_integral = PID_MAX_INTEGRAL;
        else if (p->m_integral < -PID_MAX_INTEGRAL)
        p->m_integral = -PID_MAX_INTEGRAL;

        // calculate PID term
        pid = (error*p->m_pgain + ((p->m_integral*p->m_igain)>>4) + (error - p->m_prevError)*p->m_dgain) >> 10;

        if (p->m_servo) {
        p->m_command += pid; // since servo is a position device, we integrate the pid term
        if (p->m_command > PIXY_RCS_MAX_POS) 
            p->m_command = PIXY_RCS_MAX_POS; 
        else if (p->m_command < PIXY_RCS_MIN_POS) 
            p->m_command = PIXY_RCS_MIN_POS;
        } else {
            // Deal with Zumo base deadband
            if (p->pid > 0)
                p->pid += ZUMO_BASE_DEADBAND;
            else if (pid<0)
                p->pid -= ZUMO_BASE_DEADBAND;
            p->m_command = pid; // Zumo base is velocity device, use the pid term directly  
        }
    }

    // retain the previous error val so we can calc the derivative
    p->m_prevError = error; 
}
