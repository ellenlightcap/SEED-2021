void Controller() {
   float rho_dot_error=0;
   float phi_dot_error=0;
   float PWM_bar=0;
   float PWM_delta = 0;
   int M1PWM_value=0;
   int M2PWM_value=0;
   float rho_error=0;
   float rho_error_old = 0;
   float phi_error=0;
   float phi_error_old=0;
   float rho_dot_setpoint_internal=0;
   float phi_dot_setpoint_internal=0;

   // set velocity setpoints to global variables in case velocity control is desired
   rho_dot_setpoint_internal = rho_dot_setpoint;
   phi_dot_setpoint_internal = phi_dot_setpoint;
   Ki_rho_dot_use = Ki_rho_dot;
   Ki_phi_dot_use = Ki_phi_dot;

   // implement outer controllers to set velocity setpoint, if position control is desired
   if (ANGULAR_POSITION_CONTROL) {
        phi_error = phi_setpoint - phi;
        I_phi += phi_error*(float)period/1000.0;
        phi_dot_setpoint= Kp_phi*phi_error + Kd_phi*(phi_error- phi_error_old)/((float)period/1000) + Ki_phi*I_phi;
        phi_error_old = phi_error;
        Ki_rho_dot_use = 0;
   }
   if (POSITION_CONTROL) {
         rho_error = rho_setpoint - rho;
         I_rho += rho_error*(float)period/1000.0
         rho_dot_setpoint = Kp_rho*rho_error + Kd_rho*(rho_error - rho_error_old)/((float)period/1000) + Ki_rho*I_rho;
         rho_error_old = rho_error;
         Ki_phi_dot_use = 0;
   }

   // Inner Control Loop
   rho_dot_error = rho_dot_setpoint - rho_dot;
   phi_dot_error = phi_dot_setpoint - phi_dot;

   I_rho_dot = I_rho_dot + rho_dot_error*(float)period/1000.0;
   I_phi_dot = I_phi_dot + phi_dot_error*(float)period/1000.0;

   PWM_bar = Kp_rho_dot*rho_dot_error + Ki_rho_dot_use*I_rho_dot;
   PWM_delta = Kp_phi_dot*phi_dot_error  + Ki_phi_dot_use*I_phi_dot;

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
    I_phi=0;  
    I_rho=0;  
    }
  if (M2PWM_value>255) {
    M2PWM_value =255;
    I_rho_dot=0;
    I_phi_dot=0;
    I_phi=0;  
    I_rho=0;  
  }
  analogWrite(M1PWM, M1PWM_value);
  analogWrite(M2PWM, M2PWM_value); 
  
 
  Serial.print(M1PWM_value);
  Serial.print("\t");
  Serial.print(M2PWM_value);
  Serial.print("\t");  
}
