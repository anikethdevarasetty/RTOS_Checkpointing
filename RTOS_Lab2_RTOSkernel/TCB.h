struct tcb{
    int32_t *thread_sp; //Do we want the stack inside the TCB or outside?
    struct tcb *next_TCB; 
    struct tcb *prev_TCB; //We need to decide if we want a doubly linked list
    uint8_t sleep_state;
	  uint8_t id;
    uint8_t priority;
    uint8_t blocked_state;
		uint8_t killed;
};
typedef struct tcb TCB_t;