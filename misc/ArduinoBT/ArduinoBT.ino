#define MAX_RPM  15000
#define MAX_VEL  180
#define MAX_GEAR 6
#define MAX_CNT  MAX_RPM
#define RPM_STP  40
#define VEL_STP  20
#define GEAR_STP 40

int rpm = 0, velocity = 0, gear = 0, rpm_counter = 0, velo_counter = 0, gear_counter = 0;

void setup() {
  Serial.begin(9600);
}

void loop() { 
  rpm      %= MAX_RPM;
  velocity %= MAX_VEL;
  gear     %= MAX_GEAR;
  
  Serial.print('r');
  Serial.print(rpm);
  Serial.print('v');
  Serial.print(velocity);  
  Serial.print('g');
  Serial.print(gear);
  Serial.println('e');

  if(rpm_counter  > (MAX_CNT/MAX_RPM)) {
    rpm++;
    rpm_counter = 0;
  }
  if(velo_counter > (MAX_CNT/MAX_VEL)) {
    velocity++;
    velo_counter = 0;
  }
  if(gear_counter > (MAX_CNT/MAX_GEAR)) {  
    gear++;
    gear_counter = 0;
  }
  
  rpm_counter  += RPM_STEP;
  velo_counter += VEL_STEP;
  gear_counter += GEAR_STEP;
}
