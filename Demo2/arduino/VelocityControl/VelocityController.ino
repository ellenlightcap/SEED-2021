void VelocityController() {
   float rho_dot_error=0;
   float phi_dot_error=0;
   float PWM_bar=0;
   float PWM_delta = 0;
   int M1PWM_value=0;
   int M2PWM_value=0;
   
   rho_dot_error = rho_dot_setpoint - rho_dot;
   phi_dot_error = phi_dot_setpoint - phi_dot;

   I_rho_dot = I_rho_dot + rho_dot_error*(float)period/1000.0;
   I_phi_dot = I_phi_dot + phi_dot_error*(float)period/1000.0;

   PWM_bar = Kp_rho_dot*rho_dot_error + Ki_rho_dot*I_rho_dot;
   PWM_delta = Kp_phi_dot*phi_dot_error  + Ki_phi_dot*I_phi_dot;

   M1PWM_value = (PWM_bar+PWM_delta)/2;
   M2PWM_value = (PWM_bar-PWM_delta)/2;

  if (M1PWM_value>0) {
    digitalWrite(M1DIR,LOW);
  }
  else{
    digitalWrite(M1DIR,HIGH);
  }

  if (M2PWM_value>0) {
    digitalWrite(M2DIR,HIGH);
  }
  else{
    digitalWrite(M2DIR,LOW);
  }
  
  M1PWM_value = abs(M1PWM_value);
  M2PWM_value = abs(M2PWM_value);

  if (M1PWM_value>255) {
    M1PWM_value =255;
    I_rho_dot=0;
    I_phi_dot=0;
  }
  if (M2PWM_value>255) {
    M2PWM_value =255;
    I_rho_dot=0;
    I_phi_dot=0;
  }
  analogWrite(M1PWM, M1PWM_value);
  analogWrite(M2PWM, M2PWM_value); 
  
   
}
