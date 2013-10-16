
int macro0() {
//Wall-follower
  int state = NO_REFERENCE;	
  const int num_children = 2;
  static int init = TRUE;
  static int* child_states;
  int i;
  
  if (init) {
    child_states = (int*) malloc(num_children*sizeof(int));
    for (i=0; i<num_children; i++) {
      child_states[i] = DONT_CARE;
    }
    init = FALSE;
  }
  
  //                     Avoid-Wall-Ahead    Hug-Side-Wall
  int stateNum = states_to_int(child_states, num_children);
  
  if (stateNum == 5) {  //Initial condition
    child_states[0] = primitive0();
    child_states[1] = primitive1();
    state = NO_REFERENCE;
  }
  else if (stateNum % 4 == TRANSIENT) {  //There is a wall ahead
    child_states[0] = primitive0();
    child_states[1] = DONT_CARE;
    state = TRANSIENT;
  }
  else if (stateNum < 12) {
    child_states[0] = primitive0();
    child_states[1] = primitive1();
    state = TRANSIENT;
  }
  else {
    child_states[0] = primitive0();
    child_states[1] = primitive1();
    state = CONVERGED;
  }
  
//  Serial.println(stateNum);
  
  return state;
}

int states_to_int(int* states, int num_states) {
  int i;
  int mult = 1;
  int composite_state = 0;
  for (i = 0; i < num_states; i++) {
    composite_state += states[i] * mult;
    mult *= 4;		
  }
  return composite_state;
}

