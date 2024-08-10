
void rf_ticks() {

  // Read the value for the encoder for the right front wheel
  int val = digitalRead(right_front_enB);

  //when wheel is moving in cw direction , output a is ahead of output b and is high(rising) while output b is low
  if (val == LOW) {
    Direction_rf = false;  // Reverse
  } else {
    Direction_rf = true;  // Forward
  }

  //if direction_rf is true (moving forward)
  if (Direction_rf) {
    
    //if the tick count has reached the maximum value (encoder maximum), it wraps around to minimum value (encoder minimum)
    if (rf_ticks_count == encoder_maximum) {
      rf_ticks_count = encoder_minimum;

    //updates ticks 
    } else {
      rf_ticks_count--;
      rf_ticks_pub--;
    }
  } else {
    if (rf_ticks_count == encoder_minimum) {
      rf_ticks_count = encoder_maximum;
    } else {
      rf_ticks_count++;
      rf_ticks_pub++;
    }
  }
}

void rb_ticks() {

  // Read the value for the encoder for the right back wheel
  int val = digitalRead(right_back_enB);

  if (val == LOW) {
    Direction_rb = false;  // Reverse
  } else {
    Direction_rb = true;  // Forward
  }

  if (Direction_rb) {

    if (rb_ticks_count == encoder_maximum) {
      rb_ticks_count = encoder_minimum;
    } else {
      rb_ticks_count--;
      rb_ticks_pub--;
    }
  } else {
    if (rb_ticks_count == encoder_minimum) {
      rb_ticks_count = encoder_maximum;
    } else {
      rb_ticks_count++;
      rb_ticks_pub++;
    }
  }
}
void lf_ticks() {

  // Read the value for the encoder for the left front wheel
  int val = digitalRead(left_front_enB);

  if (val == LOW) {
    Direction_lf = false;  // Reverse
  } else {
    Direction_lf = true;  // Forward
  }

  if (Direction_lf) {

    if (lf_ticks_count == encoder_maximum) {
      lf_ticks_count = encoder_minimum;
    } else {
      lf_ticks_count++;
      lf_ticks_pub++;
    }
  } else {
    if (lf_ticks_count == encoder_minimum) {
      lf_ticks_count = encoder_maximum;
    } else {
      lf_ticks_count--;
      lf_ticks_pub--;
    }
  }
}
void lb_ticks() {

  // Read the value for the encoder for the left back wheel
  int val = digitalRead(left_back_enB);

  if (val == LOW) {
    Direction_lb = false;  // Reverse
  } else {
    Direction_lb = true;  // Forward
  }

  if (Direction_lb) {

    if (lb_ticks_count == encoder_maximum) {
      lb_ticks_count = encoder_minimum;
    } else {
      lb_ticks_count++;
      lb_ticks_pub++;
    }
  } else {
    if (lb_ticks_count == encoder_minimum) {
      lb_ticks_count = encoder_maximum;
    } else {
      lb_ticks_count--;
      lb_ticks_pub--;
    }
  }
}
