struct tcb{
    int32_t *thread_sp; //Do we want the stack inside the TCB or outside?
    struct tcb *next_TCB; 
    struct tcb *prev_TCB; //We need to decide if we want a doubly linked list
		int32_t *stack_top;
    uint8_t sleep_state;
	  uint8_t id;
    uint8_t priority;
    uint8_t blocked_state;
		uint8_t killed;
};
typedef struct tcb TCB_t;

#define STACKSIZE  100      // number of 32-bit words in stack

struct stack{
  int32_t buffer[STACKSIZE];
};
typedef struct stack stack_t;