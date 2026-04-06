#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ##################################################################################################################
// ATTENTION: THE CODE IS COMPLETELY WRITTEN IN ARDUINO STYLE BECAUSE OUR STM32 GOT BURNT 1 DAY BEFORE THE DEMO
// SO WE HAD TO USE WHAT WE HAD IN OUR HANDS.
// ##################################################################################################################

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ═══════════════════════════════════════════════════
//  PCA9685 Channels
// ═══════════════════════════════════════════════════
#define CH_GRIPPER   0   // no encoder
#define CH_WRIST_R   1   // no encoder
#define CH_WRIST_P   2   // encoder A3
#define CH_FOREARM   3   // no encoder
#define CH_ELBOW     7   // encoder A2
#define CH_SHOULDER  5   // encoder A1
#define CH_BASE      6   // encoder A0
#define NUM_CHANNELS 8

// ═══════════════════════════════════════════════════
//  Encoder Wiring
//
//  Pot center tap → analog pin
//  Pot outer legs → GND and Arduino 5V
//
//  A0 = Base        (CH6)
//  A1 = Shoulder    (CH5)
//  A2 = Elbow       (CH7)
//  A3 = Wrist Pitch (CH2)
//  A4 = SDA (I2C — taken)
//  A5 = SCL (I2C — taken)
// ═══════════════════════════════════════════════════
#define NUM_ENCODERS 4

static const uint8_t ENC_PIN[NUM_ENCODERS]   = { A0,     A1,         A2,       A3 };
static const uint8_t ENC_CH[NUM_ENCODERS]    = { CH_BASE, CH_SHOULDER, CH_ELBOW, CH_WRIST_P };

static void printEncName(uint8_t e) {
  switch(e) {
    case 0: Serial.print(F("base")); break;
    case 1: Serial.print(F("shld")); break;
    case 2: Serial.print(F("elb ")); break;
    case 3: Serial.print(F("wrP ")); break;
  }
}

// Encoder calibration: ADC values at two known servo positions
// ── Run 'C' command to measure these, then update here ──
  static int ENC_ADC_LOW[4]  = { 296, 296, 299, 298 };
  static int ENC_ADC_HIGH[4] = { 587, 584, 592, 591 };
static float SV_CAL_LOW[NUM_ENCODERS]  = { 60,  60,  60,  60 };
static float SV_CAL_HIGH[NUM_ENCODERS] = { 150, 150, 150, 150 };

// ═══════════════════════════════════════════════════
//  PWM / Servo
// ═══════════════════════════════════════════════════
#define SERVO_MIN    150
#define SERVO_MAX    600
#define SERVO_FREQ    50
#define SERVO_LIMIT  175

#define INTERP_STEP_DEG  2
#define INTERP_DELAY_MS  15

// ═══════════════════════════════════════════════════
//  Closed-Loop Settings
// ═══════════════════════════════════════════════════
#define CL_TOLERANCE   2.5f   // degrees — close enough
#define CL_MAX_ITER    30     // max correction loops
#define CL_SETTLE_MS   40     // settle time between corrections
#define STALL_THRESH   0.5f   // degrees — if movement < this, stalled
#define STALL_MAX      4      // stall readings before giving up

// ═══════════════════════════════════════════════════
//  Software Joint Limits
// ═══════════════════════════════════════════════════
static const int SV_MIN[NUM_CHANNELS] = { 60, 10, 10, 10, 10, 10, 10, 10 };
static const int SV_MAX[NUM_CHANNELS] = { 170, 170, 170, 170, 170, 170, 170, 170 };

static int currentSV[NUM_CHANNELS];

// ═══════════════════════════════════════════════════
//  DH Parameters (cm)
// ═══════════════════════════════════════════════════
static const float DH_a[6]     = { -1.0f, 10.5f, 0.0f, 0.0f, 0.0f, 0.0f };
static const float DH_d[6] = { 9.0f, 0.0f, 0.0f, 14.4f, 0.0f, 18.0f };
static const float DH_alpha[6] = {
  (float)(M_PI/2.0), 0.0f, (float)(M_PI/2.0),
  (float)(-M_PI/2.0), (float)(M_PI/2.0), 0.0f
};
#define A1 DH_a[0]
#define A2 DH_a[1]
#define A3 DH_a[2]
#define D1 DH_d[0]
#define D4 DH_d[3]
#define D6 DH_d[5]

static const float SERVO_SCALE = 1.0f;  // 180° servo, 1:1 mapping

// ═══════════════════════════════════════════════════
//  Calibration
// ═══════════════════════════════════════════════════
static const int HOME_SV[NUM_CHANNELS] = { 160, 170, 80, 90, 90, 90, 80, 90 };
static float HOME_DH[6] = { 0.0f, 95.4f, -19.2f, 0.0f, 13.9f, 0.0f };
static const float DIR[6] = { +1, +1, +1, +1, -1, +1 };
static const uint8_t JOINT_CH[6] = {
  CH_BASE, CH_SHOULDER, CH_ELBOW, CH_FOREARM, CH_WRIST_P, CH_WRIST_R
};

// ═══════════════════════════════════════════════════
//  Matrix Math
// ═══════════════════════════════════════════════════
typedef float Mat3[3][3];

static void mat3_identity(Mat3 R) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R[i][j] = (i == j) ? 1.0f : 0.0f;
}

static void mat3_mul(const Mat3 A, const Mat3 B, Mat3 C) {
  float tmp[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      tmp[i][j] = 0;
      for (int k = 0; k < 3; k++)
        tmp[i][j] += A[i][k] * B[k][j];
    }
  memcpy(C, tmp, sizeof(tmp));
}

static void mat3_mulTA(const Mat3 A, const Mat3 B, Mat3 C) {
  float tmp[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      tmp[i][j] = 0;
      for (int k = 0; k < 3; k++)
        tmp[i][j] += A[k][i] * B[k][j];
    }
  memcpy(C, tmp, sizeof(tmp));
}

static void mat3_print(const Mat3 R) {
  for (int i = 0; i < 3; i++) {
    Serial.print(F("  ["));
    for (int j = 0; j < 3; j++) {
      if (R[i][j] >= 0) Serial.print(' ');
      Serial.print(R[i][j], 3);
      if (j < 2) Serial.print(F(", "));
    }
    Serial.println(F("]"));
  }
}

static void dh_rotation(float theta, float alpha, Mat3 R) {
  float ct = cosf(theta), st = sinf(theta);
  float ca = cosf(alpha), sa = sinf(alpha);
  R[0][0] = ct;   R[0][1] = -st*ca; R[0][2] =  st*sa;
  R[1][0] = st;   R[1][1] =  ct*ca; R[1][2] = -ct*sa;
  R[2][0] = 0.0f; R[2][1] =  sa;    R[2][2] =  ca;
}

static void computeR03(float t1, float t2, float t3, Mat3 R03) {
  Mat3 R01, R12, R23, tmp;
  dh_rotation(t1, DH_alpha[0], R01);
  dh_rotation(t2, DH_alpha[1], R12);
  dh_rotation(t3, DH_alpha[2], R23);
  mat3_mul(R01, R12, tmp);
  mat3_mul(tmp, R23, R03);
}

void forwardKinematics(const float q[6], float p[3], Mat3 Ree) {
  float pos[3] = {0, 0, 0};
  Mat3 Rcur;
  mat3_identity(Rcur);
  for (int i = 0; i < 6; i++) {
    Mat3 Ri;
    dh_rotation(q[i], DH_alpha[i], Ri);
    float ct = cosf(q[i]), st = sinf(q[i]);
    float ti[3] = { DH_a[i]*ct, DH_a[i]*st, DH_d[i] };
    for (int r = 0; r < 3; r++) {
      float s = 0;
      for (int c = 0; c < 3; c++) s += Rcur[r][c] * ti[c];
      pos[r] += s;
    }
    Mat3 tmp;
    mat3_mul(Rcur, Ri, tmp);
    memcpy(Rcur, tmp, sizeof(Mat3));
  }
  p[0] = pos[0]; p[1] = pos[1]; p[2] = pos[2];
  memcpy(Ree, Rcur, sizeof(Mat3));
}

// ═══════════════════════════════════════════════════
//  Encoder Functions
// ═══════════════════════════════════════════════════

// Read with 8-sample averaging
static int readEncoderRaw(uint8_t enc) {
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(ENC_PIN[enc]);
    delayMicroseconds(250);
  }
  return (int)(sum / 8);
}

// ADC → servo degrees (linear interpolation from calibration)
static float encoderToDeg(uint8_t enc, int adc) {
  float frac = (float)(adc - ENC_ADC_LOW[enc]) /
               (float)(ENC_ADC_HIGH[enc] - ENC_ADC_LOW[enc]);
  return SV_CAL_LOW[enc] + frac * (SV_CAL_HIGH[enc] - SV_CAL_LOW[enc]);
}

// Read actual servo position in degrees
static float readActualDeg(uint8_t enc) {
  return encoderToDeg(enc, readEncoderRaw(enc));
}

// Find encoder index for a PCA9685 channel, -1 if none
static int8_t encForCh(uint8_t ch) {
  for (int i = 0; i < NUM_ENCODERS; i++)
    if (ENC_CH[i] == ch) return i;
  return -1;
}

// ═══════════════════════════════════════════════════
//  Servo Drivers
// ═══════════════════════════════════════════════════
static uint16_t degToPulse(int deg) {
  return (uint16_t)map(constrain(deg, 0, SERVO_LIMIT), 0, 180, SERVO_MIN, SERVO_MAX);
}

static void setServoRaw(uint8_t ch, int deg) {
  deg = constrain(deg, SV_MIN[ch], SV_MAX[ch]);
  pwm.setPWM(ch, 0, degToPulse(deg));
  currentSV[ch] = deg;
}

// Open-loop smooth (for non-encoded channels)
static void setServoSmooth(uint8_t ch, int target) {
  target = constrain(target, SV_MIN[ch], SV_MAX[ch]);
  int cur = currentSV[ch];
  if (cur == target) return;
  int d = (target > cur) ? 1 : -1;
  while (cur != target) {
    int step = min((int)INTERP_STEP_DEG, abs(target - cur));
    cur += d * step;
    pwm.setPWM(ch, 0, degToPulse(cur));
    delay(INTERP_DELAY_MS);
  }
  currentSV[ch] = target;
}

// ═══════════════════════════════════════════════════
//  CLOSED-LOOP SINGLE SERVO
//
//  currentSV = the actual PWM command being sent.
//  This ensures next ramp starts from where the
//  servo actually is, not where we wished it was.
// ═══════════════════════════════════════════════════
static bool setServoCL(uint8_t ch, int target) {
  int8_t enc = encForCh(ch);
  if (enc < 0) {
    setServoSmooth(ch, target);
    return true;
  }

  target = constrain(target, SV_MIN[ch], SV_MAX[ch]);

  // Phase 1: open-loop ramp to target
  setServoSmooth(ch, target);
  delay(100);

  // Phase 2: closed-loop — keep adjusting PWM until
  // encoder reads within tolerance of the target
  int cmd = target;
  int stalls = 0;
  float prev = readActualDeg(enc);

  for (int i = 0; i < CL_MAX_ITER; i++) {
    float actual = readActualDeg(enc);
    float err = (float)target - actual;

    if (fabsf(err) <= CL_TOLERANCE) {
      currentSV[ch] = cmd;  // store actual PWM
      return true;
    }

    if (fabsf(actual - prev) < STALL_THRESH) {
      stalls++;
      if (stalls >= STALL_MAX) {
        Serial.print(F("  STALL CH")); Serial.print(ch);
        Serial.print(F(" tgt=")); Serial.print(target);
        Serial.print(F(" act=")); Serial.println(actual, 1);
        currentSV[ch] = cmd;
        return false;
      }
    } else { stalls = 0; }
    prev = actual;

    int nudge = (int)(err * 0.6f);
    if (nudge == 0) nudge = (err > 0) ? 1 : -1;
    cmd = constrain(cmd + nudge, SV_MIN[ch], SV_MAX[ch]);
    pwm.setPWM(ch, 0, degToPulse(cmd));
    currentSV[ch] = cmd;
    delay(CL_SETTLE_MS);
  }

  float fin = readActualDeg(enc);
  Serial.print(F("  TIMEOUT CH")); Serial.print(ch);
  Serial.print(F(" tgt=")); Serial.print(target);
  Serial.print(F(" act=")); Serial.println(fin, 1);
  currentSV[ch] = cmd;
  return false;
}

// ═══════════════════════════════════════════════════
//  MULTI-SERVO MOVE + CLOSED-LOOP
//
//  Phase 1: synchronized ramp from current to target
//  Phase 2: per-joint correction using encoders
//  currentSV always reflects actual PWM being sent
// ═══════════════════════════════════════════════════
static void setServosSmooth(const uint8_t* chs, const int* targets, int n) {
  int clamped[8], starts[8];
  int maxTravel = 0;
  for (int i = 0; i < n; i++) {
    clamped[i] = constrain(targets[i], SV_MIN[chs[i]], SV_MAX[chs[i]]);
    starts[i] = currentSV[chs[i]];
    int travel = abs(clamped[i] - starts[i]);
    if (travel > maxTravel) maxTravel = travel;
  }

  // Phase 1: synchronized open-loop ramp (skip if nothing to move)
  if (maxTravel > 0) {
    int numSteps = (maxTravel + INTERP_STEP_DEG - 1) / INTERP_STEP_DEG;
    for (int s = 1; s <= numSteps; s++) {
      float frac = (float)s / (float)numSteps;
      for (int i = 0; i < n; i++) {
        int pos = starts[i] + (int)((clamped[i] - starts[i]) * frac + 0.5f);
        pwm.setPWM(chs[i], 0, degToPulse(pos));
      }
      delay(INTERP_DELAY_MS);
    }
    for (int i = 0; i < n; i++)
      currentSV[chs[i]] = clamped[i];
  }

  // Phase 2: ALWAYS run correction (even if ramp was skipped)
  delay(120);
  for (int i = 0; i < n; i++) {
    int8_t enc = encForCh(chs[i]);
    if (enc < 0) continue;

    int cmd = currentSV[chs[i]];
    for (int a = 0; a < 20; a++) {
      float actual = readActualDeg(enc);
      float err = (float)clamped[i] - actual;

      if (fabsf(err) <= CL_TOLERANCE) break;

      int nudge = (int)(err * 0.6f);
      if (nudge == 0) nudge = (err > 0) ? 1 : -1;
      cmd = constrain(cmd + nudge, SV_MIN[chs[i]], SV_MAX[chs[i]]);
      pwm.setPWM(chs[i], 0, degToPulse(cmd));
      delay(CL_SETTLE_MS);
    }
    currentSV[chs[i]] = cmd;  // store actual PWM sent
  }
}

// ═══════════════════════════════════════════════════
//  IK / FK
// ═══════════════════════════════════════════════════
struct IKResult { bool valid; float q[6]; int sv[6]; };

static void buildRd_horizontal(float xd, float yd, Mat3 Rd) {
  float t1 = atan2f(yd, xd);
  float c1 = cosf(t1), s1 = sinf(t1);
  Rd[0][0]=0; Rd[0][1]= s1; Rd[0][2]=c1;
  Rd[1][0]=0; Rd[1][1]=-c1; Rd[1][2]=s1;
  Rd[2][0]=1; Rd[2][1]=  0; Rd[2][2]= 0;
}

static int dhToServo(int joint, float dh_deg) {
  uint8_t ch = JOINT_CH[joint];
  float delta = (dh_deg - HOME_DH[joint]) * DIR[joint] * SERVO_SCALE;
  return constrain((int)(HOME_SV[ch] + delta + 0.5f), SV_MIN[ch], SV_MAX[ch]);
}

IKResult solveIK(float xd, float yd, float zd, const Mat3 Rd) {
  IKResult res; res.valid = false;
  float xc = xd - D6*Rd[0][2], yc = yd - D6*Rd[1][2], zc = zd - D6*Rd[2][2];

  float r_ee = sqrtf(xd*xd + yd*yd);
  if (r_ee < D6 && fabsf(Rd[2][2]) < 0.01f) {
    Serial.println(F("ERR: Too close")); return res;
  }

  float theta1 = atan2f(yc, xc);
  float r = sqrtf(xc*xc+yc*yc), s = zc-D1, dr = r-A1;
  float t = sqrtf(dr*dr+s*s), u = sqrtf(A3*A3+D4*D4), phi = atan2f(A3,D4);

  if (t > A2+u+0.01f) { Serial.println(F("ERR: Too far")); return res; }
  if (t < fabsf(A2-u)-0.01f) { Serial.println(F("ERR: Too close")); return res; }

  float Dc = constrain((t*t-A2*A2-u*u)/(2.0f*A2*u), -1.0f, 1.0f);
  float theta3 = atan2f(Dc, sqrtf(1.0f-Dc*Dc)) - phi;
  float alpha_a = atan2f(s, dr);
  float beta = atan2f(-u*cosf(theta3+phi), u*sinf(theta3+phi)+A2);
  float theta2 = alpha_a - beta;

  Mat3 R03, R36;
  computeR03(theta1, theta2, theta3, R03);
  mat3_mulTA(R03, Rd, R36);

  float theta4, theta5, theta6;
  float m33 = constrain(R36[2][2], -1.0f, 1.0f);

  if (fabsf(m33-1.0f) < 1e-4f) {
    theta5=0; theta4=0; theta6=atan2f(R36[1][0],R36[0][0]);
  } else if (fabsf(m33+1.0f) < 1e-4f) {
    theta5=(float)M_PI; theta4=0; theta6=-atan2f(-R36[1][0],-R36[0][0]);
  } else {
    theta5 = atan2f(sqrtf(1.0f-m33*m33), m33);
    theta4 = atan2f(R36[1][2], R36[0][2]);
    theta6 = atan2f(R36[2][1], -R36[2][0]);
  }

  // ZYZ flip
  { float t4d=theta4*57.2958f, t6d=theta6*57.2958f;
    if (fabsf(t4d)>90||fabsf(t6d)>90) {
      theta4 += (theta4>=0) ? -(float)M_PI : (float)M_PI;
      theta5 = -theta5;
      theta6 += (theta6>=0) ? -(float)M_PI : (float)M_PI;
    }
  }

  float qr[6] = {theta1,theta2,theta3,theta4,theta5,theta6};
  for (int i=0;i<6;i++) {
    res.q[i] = qr[i]*57.2958f;
    res.sv[i] = dhToServo(i, res.q[i]);
  }
  res.valid = true;
  return res;
}

static void printIK(const IKResult& ik) {
  Serial.println(F("-------------------------------"));
  for (int i=0;i<6;i++) {
    Serial.print(F("  "));
    switch(i) {
      case 0: Serial.print(F("base    ")); break;
      case 1: Serial.print(F("shoulder")); break;
      case 2: Serial.print(F("elbow   ")); break;
      case 3: Serial.print(F("forearm ")); break;
      case 4: Serial.print(F("wristP  ")); break;
      case 5: Serial.print(F("wristR  ")); break;
    }
    Serial.print(F(" DH=")); Serial.print(ik.q[i],1);
    Serial.print(F("  sv=")); Serial.println(ik.sv[i]);
  }
  Serial.println(F("-------------------------------"));
}

// ═══════════════════════════════════════════════════
//  Move with closed-loop
// ═══════════════════════════════════════════════════
bool moveTo(float x, float y, float z) {
  Mat3 Rd; buildRd_horizontal(x, y, Rd);
  IKResult ik = solveIK(x, y, z, Rd);
  if (!ik.valid) return false;
  printIK(ik);

  int baseDelta = abs(ik.sv[0] - currentSV[CH_BASE]);
  if (baseDelta > 5) {
    Serial.println(F("  [retract]"));
    uint8_t rc[2]={CH_SHOULDER,CH_ELBOW}; int rt[2]={HOME_SV[CH_SHOULDER],HOME_SV[CH_ELBOW]};
    setServosSmooth(rc,rt,2); delay(200);
    Serial.println(F("  [rotate]"));
    setServoCL(CH_BASE, ik.sv[0]); delay(200);
    Serial.println(F("  [extend]"));
    uint8_t ec[5]={CH_SHOULDER,CH_ELBOW,CH_FOREARM,CH_WRIST_P,CH_WRIST_R};
    int et[5]={ik.sv[1],ik.sv[2],ik.sv[3],ik.sv[4],ik.sv[5]};
    setServosSmooth(ec,et,5);
  } else {
    Serial.println(F("  [direct]"));
    uint8_t ac[6]={CH_BASE,CH_SHOULDER,CH_ELBOW,CH_FOREARM,CH_WRIST_P,CH_WRIST_R};
    int at_[6]={ik.sv[0],ik.sv[1],ik.sv[2],ik.sv[3],ik.sv[4],ik.sv[5]};
    setServosSmooth(ac,at_,6);
  }

  // Print actual positions from encoders vs IK targets
  Serial.println(F("  [encoder verify]"));
  // Map encoder index to IK servo target
  for (int e=0; e<NUM_ENCODERS; e++) {
    float act = readActualDeg(e);
    // Find which IK joint this encoder corresponds to
    int ikTgt = 0;
    uint8_t ch = ENC_CH[e];
    for (int j=0; j<6; j++) { if (JOINT_CH[j]==ch) { ikTgt=ik.sv[j]; break; } }
    Serial.print(F("    ")); printEncName(e);
    Serial.print(F(" ik=")); Serial.print(ikTgt);
    Serial.print(F(" pwm=")); Serial.print(currentSV[ch]);
    Serial.print(F(" act=")); Serial.print(act, 1);
    Serial.print(F(" err=")); Serial.println(fabsf(ikTgt - act), 1);
  }
  return true;
}

// ═══════════════════════════════════════════════════
//  Home
// ═══════════════════════════════════════════════════
void goHome() {
  Serial.println(F("-> HOME"));
  uint8_t rc[2]={CH_SHOULDER,CH_ELBOW}; int rt[2]={HOME_SV[CH_SHOULDER],HOME_SV[CH_ELBOW]};
  setServosSmooth(rc,rt,2); delay(200);
  setServoCL(CH_BASE, HOME_SV[CH_BASE]); delay(200);
  uint8_t oc[4]={CH_FOREARM,CH_WRIST_P,CH_WRIST_R,CH_GRIPPER};
  int ot[4]={HOME_SV[CH_FOREARM],HOME_SV[CH_WRIST_P],HOME_SV[CH_WRIST_R],HOME_SV[CH_GRIPPER]};
  setServosSmooth(oc,ot,4);

  float qr[6]; for(int i=0;i<6;i++) qr[i]=HOME_DH[i]*0.0174533f;
  float p[3]; Mat3 R; forwardKinematics(qr,p,R);
  Serial.print(F("  Home FK: (")); Serial.print(p[0],1);
  Serial.print(F(", ")); Serial.print(p[1],1);
  Serial.print(F(", ")); Serial.print(p[2],1); Serial.println(F(")"));
  Serial.println(F("-> HOME done."));
}

// ═══════════════════════════════════════════════════
//  Parsers
// ═══════════════════════════════════════════════════
static bool parseXYZ(const String& s, float &x, float &y, float &z) {
  String rem=s.substring(1); rem.trim(); rem.replace(',',' ');
  int p1=rem.indexOf(' '); if(p1<0)return false;
  int p2=rem.indexOf(' ',p1+1); if(p2<0)return false;
  x=rem.substring(0,p1).toFloat();
  y=rem.substring(p1+1,p2).toFloat();
  z=rem.substring(p2+1).toFloat();
  return true;
}

static int parseFloats(const String& s, int offset, float* v, int maxN) {
  String rem=s.substring(offset); rem.trim(); rem.replace(',',' ');
  int count=0, pos=0;
  while(count<maxN && pos<(int)rem.length()) {
    int sp=rem.indexOf(' ',pos); if(sp<0)sp=rem.length();
    if(sp>pos) v[count++]=rem.substring(pos,sp).toFloat();
    pos=sp+1;
  }
  return count;
}

// ═══════════════════════════════════════════════════
//  Startup Info  (also callable via '?' command)
// ═══════════════════════════════════════════════════
static void printStartupInfo() {
  Serial.println();
  Serial.println(F("====== 6-DOF IK v4 (Encoders) ======"));
  Serial.println(F("  H            home"));
  Serial.println(F("  M x y z      move (closed-loop)"));
  Serial.println(F("  D x y z      dry run"));
  Serial.println(F("  F t1..t6     FK"));
  Serial.println(F("  J joint deg  set one joint"));
  Serial.println(F("  G 0/1        gripper"));
  Serial.println(F("  S ch ang     raw servo"));
  Serial.println(F("  P            positions + encoders"));
  Serial.println(F("  C            encoder calibration"));
  Serial.println(F("  E            read all encoders now"));
  Serial.println(F("  R            re-init PCA9685"));
  Serial.println(F("  T ch         test wiggle"));
  Serial.println(F("  ?            print this menu + status"));
  Serial.println(F("====================================="));

  // Show encoder readings
  Serial.println(F("  Encoder readings:"));
  for (int e = 0; e < NUM_ENCODERS; e++) {
    int adc = readEncoderRaw(e);
    float deg = encoderToDeg(e, adc);
    Serial.print(F("    ")); printEncName(e);
    Serial.print(F(" ADC=")); Serial.print(adc);
    Serial.print(F(" deg=")); Serial.println(deg, 1);
  }

  // Show home FK
  float qr[6]; for (int i = 0; i < 6; i++) qr[i] = HOME_DH[i] * 0.0174533f;
  float p[3]; Mat3 R; forwardKinematics(qr, p, R);
  Serial.print(F("  Home FK: (")); Serial.print(p[0], 1);
  Serial.print(F(", ")); Serial.print(p[1], 1);
  Serial.print(F(", ")); Serial.print(p[2], 1); Serial.println(F(")"));
  Serial.println(F("-> Ready. Run 'C' to calibrate encoders."));
}

// ═══════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  delay(500);

  // Configure encoder pins
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(200);
  Serial.println(F("  PCA9685 OK"));

  for (int i=0; i<NUM_CHANNELS; i++) currentSV[i] = HOME_SV[i];

  for (int i=0; i<NUM_CHANNELS; i++) {
    Serial.print(F("  CH")); Serial.print(i); Serial.print(F("->")); Serial.print(HOME_SV[i]);
    setServoRaw(i, HOME_SV[i]);
    delay(500);
    Serial.println(F(" ok"));
  }

  printStartupInfo();
}

// ═══════════════════════════════════════════════════
//  Encoder Calibration Routine
//
//  For each encoder:
//    1. Move servo to 60°, read ADC
//    2. Move servo to 130°, read ADC
//    3. Store as calibration endpoints
//
//  Run this once, note the values, update the
//  ENC_ADC_LOW/HIGH arrays in code, recompile.
// ═══════════════════════════════════════════════════
static void runCalibration() {
  Serial.println(F("=== ENCODER CALIBRATION ==="));
  Serial.println(F("  Moving each encoded joint to two positions"));
  Serial.println(F("  and reading ADC values."));
  Serial.println();

  for (int e = 0; e < NUM_ENCODERS; e++) {
    uint8_t ch = ENC_CH[e];
    Serial.print(F("  --- ")); printEncName(e);
    Serial.print(F(" (CH")); Serial.print(ch); Serial.println(F(") ---"));

    // Position A: servo = 60 (safe, away from mechanical stops)
    int posA = 60;
    Serial.print(F("    Moving to ")); Serial.print(posA); Serial.println(F("..."));
    setServoSmooth(ch, posA);
    delay(1000);  // let it settle
    int adcA = readEncoderRaw(e);
    Serial.print(F("    ADC at ")); Serial.print(posA);
    Serial.print(F(" = ")); Serial.println(adcA);

    // Position B: servo = 130
    int posB = 150;
    Serial.print(F("    Moving to ")); Serial.print(posB); Serial.println(F("..."));
    setServoSmooth(ch, posB);
    delay(1000);
    int adcB = readEncoderRaw(e);
    Serial.print(F("    ADC at ")); Serial.print(posB);
    Serial.print(F(" = ")); Serial.println(adcB);

    // Store calibration
    ENC_ADC_LOW[e] = adcA;
    ENC_ADC_HIGH[e] = adcB;
    SV_CAL_LOW[e] = (float)posA;
    SV_CAL_HIGH[e] = (float)posB;

    // Return to home
    setServoSmooth(ch, HOME_SV[ch]);
    delay(500);

    Serial.print(F("    Calibrated: ADC ")); Serial.print(adcA);
    Serial.print(F("-")); Serial.print(adcB);
    Serial.print(F(" = ")); Serial.print(posA);
    Serial.print(F("-")); Serial.print(posB); Serial.println(F(" deg"));
    Serial.println();
  }

  // Print summary for copying into code
  Serial.println(F("  === COPY INTO CODE ==="));
  Serial.print(F("  static int ENC_ADC_LOW[4]  = { "));
  for (int e=0;e<4;e++) { Serial.print(ENC_ADC_LOW[e]); if(e<3) Serial.print(F(", ")); }
  Serial.println(F(" };"));
  Serial.print(F("  static int ENC_ADC_HIGH[4] = { "));
  for (int e=0;e<4;e++) { Serial.print(ENC_ADC_HIGH[e]); if(e<3) Serial.print(F(", ")); }
  Serial.println(F(" };"));
  Serial.println(F("  ========================"));
}

// ═══════════════════════════════════════════════════
//  Loop
// ═══════════════════════════════════════════════════
static bool firstCommandReceived = false;

void loop() {
  // Re-print startup info every 3 s until the first command arrives.
  // This ensures the menu is visible regardless of when you open the terminal.
  if (!firstCommandReceived) {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 10000) {
      printStartupInfo();
      lastPrint = millis();
    }
  }

  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  firstCommandReceived = true;
  Serial.print(F("[RX] '")); Serial.print(input); Serial.println(F("'"));
  char cmd = (char)toupper((unsigned char)input[0]);

  switch (cmd) {

    case 'H': goHome(); break;

    case 'M': {
      float x,y,z;
      if (parseXYZ(input,x,y,z)) {
        Serial.print(F("Target: (")); Serial.print(x,1);
        Serial.print(F(",")); Serial.print(y,1);
        Serial.print(F(",")); Serial.print(z,1); Serial.println(F(")"));
        moveTo(x,y,z);
      } else Serial.println(F("Use: M x y z"));
      break;
    }

    case 'D': {
      float x,y,z;
      if (parseXYZ(input,x,y,z)) {
        Mat3 Rd; buildRd_horizontal(x,y,Rd);
        IKResult ik = solveIK(x,y,z,Rd);
        if (ik.valid) {
          Serial.println(F("[DRY]")); printIK(ik);
          float qr[6]; for(int i=0;i<6;i++) qr[i]=ik.q[i]*0.0174533f;
          float p[3]; Mat3 R; forwardKinematics(qr,p,R);
          Serial.print(F("  FK: (")); Serial.print(p[0],1);
          Serial.print(F(",")); Serial.print(p[1],1);
          Serial.print(F(",")); Serial.print(p[2],1); Serial.println(F(")"));
        }
      } else Serial.println(F("Use: D x y z"));
      break;
    }

    case 'F': {
      float vals[6];
      if (parseFloats(input,1,vals,6)==6) {
        float qr[6]; for(int i=0;i<6;i++) qr[i]=vals[i]*0.0174533f;
        float p[3]; Mat3 R; forwardKinematics(qr,p,R);
        Serial.println(F("[FK]"));
        Serial.print(F("  Pos: (")); Serial.print(p[0],2);
        Serial.print(F(",")); Serial.print(p[1],2);
        Serial.print(F(",")); Serial.print(p[2],2); Serial.println(F(")"));
        Serial.println(F("  Rot:")); mat3_print(R);
        for(int i=0;i<6;i++) {
          Serial.print(F("  t")); Serial.print(i+1); Serial.print(F("="));
          Serial.print(vals[i],1); Serial.print(F(" sv="));
          Serial.println(dhToServo(i,vals[i]));
        }
      } else Serial.println(F("Use: F t1 t2 t3 t4 t5 t6"));
      break;
    }

    case 'J': {
      float vals[2];
      if (parseFloats(input,1,vals,2)==2) {
        int j=(int)vals[0]-1;
        if (j>=0&&j<6) {
          int sv=dhToServo(j,vals[1]);
          Serial.print(F("J")); Serial.print(j+1);
          Serial.print(F(" DH=")); Serial.print(vals[1],1);
          Serial.print(F(" sv=")); Serial.print(sv);
          Serial.print(F(" ch=")); Serial.println(JOINT_CH[j]);
          setServoCL(JOINT_CH[j], sv);
          // Show encoder feedback if available
          int8_t enc = encForCh(JOINT_CH[j]);
          if (enc >= 0) {
            delay(100);
            float act = readActualDeg(enc);
            Serial.print(F("  encoder: ")); Serial.print(act, 1);
            Serial.print(F(" err=")); Serial.println(fabsf(sv - act), 1);
          }
          Serial.println(F("  done"));
        } else Serial.println(F("Joint 1-6"));
      } else Serial.println(F("Use: J joint deg"));
      break;
    }

    case 'G': {
      int state = input.substring(2).toInt();
      setServoSmooth(CH_GRIPPER, state ? HOME_SV[CH_GRIPPER] : 100);
      if(state) Serial.println(F("CLOSED")); else Serial.println(F("OPEN"));
      break;
    }

    case 'S': {
      int sp=input.indexOf(' ',2);
      if (sp>0) {
        int ch=input.substring(2,sp).toInt();
        int ang=input.substring(sp+1).toInt();
        if (ch>=0&&ch<NUM_CHANNELS) {
          setServoCL(ch, ang);
          Serial.print(F("  CH")); Serial.print(ch);
          Serial.print(F(" -> ")); Serial.print(currentSV[ch]);
          int8_t enc = encForCh(ch);
          if (enc >= 0) {
            delay(100);
            float act = readActualDeg(enc);
            Serial.print(F(" (actual=")); Serial.print(act, 1); Serial.print(F(")"));
          }
          Serial.println();
        } else Serial.println(F("  ERR: ch 0-6"));
      } else Serial.println(F("Use: S ch ang"));
      break;
    }

    // Positions + encoder readings
    case 'P': {
      Serial.println(F("[Positions]"));
      
      for (int i=0;i<NUM_CHANNELS;i++) {
        Serial.print(F("  CH")); Serial.print(i);
        Serial.print(F(" ")); switch(i){case 0:Serial.print(F("grip"));break;case 1:Serial.print(F("wrR "));break;case 2:Serial.print(F("wrP "));break;case 3:Serial.print(F("fore"));break;case 4:Serial.print(F("----"));break;case 5:Serial.print(F("shld"));break;case 6:Serial.print(F("base"));break;case 7:Serial.print(F("elb "));break;};
        Serial.print(F(" cmd=")); Serial.print(currentSV[i]);
        int8_t enc = encForCh(i);
        if (enc >= 0) {
          int adc = readEncoderRaw(enc);
          float act = encoderToDeg(enc, adc);
          Serial.print(F(" act=")); Serial.print(act, 1);
          Serial.print(F(" adc=")); Serial.print(adc);
          Serial.print(F(" err=")); Serial.print(fabsf(currentSV[i]-act), 1);
        } else {
          Serial.print(F(" (no enc)"));
        }
        Serial.println();
      }

      // Also show actual tip position from encoder readings
      // Build actual DH angles from encoder data
      Serial.println(F("  [Actual FK from encoders]"));
      float actualSV[6];
      for (int j = 0; j < 6; j++) {
        uint8_t ch = JOINT_CH[j];
        int8_t enc = encForCh(ch);
        if (enc >= 0) {
          actualSV[j] = readActualDeg(enc);
        } else {
          actualSV[j] = (float)currentSV[ch];  // use commanded for non-encoded
        }
      }
      // Convert servo degrees → DH degrees
      float actualDH_rad[6];
      for (int j = 0; j < 6; j++) {
        float dh_deg = HOME_DH[j] + (actualSV[j] - HOME_SV[JOINT_CH[j]]) / (DIR[j] * SERVO_SCALE);
        actualDH_rad[j] = dh_deg * 0.0174533f;
      }
      float p[3]; Mat3 R;
      forwardKinematics(actualDH_rad, p, R);
      Serial.print(F("    Actual tip: (")); Serial.print(p[0], 1);
      Serial.print(F(", ")); Serial.print(p[1], 1);
      Serial.print(F(", ")); Serial.print(p[2], 1); Serial.println(F(")"));
      break;
    }

    // Encoder calibration
    case 'C': runCalibration(); break;

    // Read encoders (live)
    case 'E': {
      Serial.println(F("[Encoders]"));
      for (int e=0; e<NUM_ENCODERS; e++) {
        int adc = readEncoderRaw(e);
        float deg = encoderToDeg(e, adc);
        Serial.print(F("  ")); printEncName(e);
        Serial.print(F(" ADC=")); Serial.print(adc);
        Serial.print(F(" deg=")); Serial.print(deg, 1);
        Serial.print(F(" cmd=")); Serial.println(currentSV[ENC_CH[e]]);
      }
      break;
    }

    // Re-init PCA9685
    case 'R':
      Serial.println(F("  Re-init..."));
      pwm.begin(); pwm.setOscillatorFrequency(27000000);
      pwm.setPWMFreq(SERVO_FREQ); delay(200);
      for (int i=0;i<NUM_CHANNELS;i++) {
        pwm.setPWM(i,0,degToPulse(currentSV[i])); delay(100);
      }
      Serial.println(F("  PCA9685 re-init done"));
      break;

    // Test wiggle
    case 'T': {
      int ch=input.substring(2).toInt();
      if (ch>=0&&ch<NUM_CHANNELS) {
        Serial.print(F("  Test CH")); Serial.println(ch);
        int home=currentSV[ch];
        int a=constrain(home-20,0,175), b=constrain(home+20,0,175);
        pwm.setPWM(ch,0,degToPulse(a)); delay(1000);
        int8_t enc = encForCh(ch);
        if (enc>=0) { Serial.print(F("  @A act=")); Serial.println(readActualDeg(enc),1); }
        pwm.setPWM(ch,0,degToPulse(b)); delay(1000);
        if (enc>=0) { Serial.print(F("  @B act=")); Serial.println(readActualDeg(enc),1); }
        pwm.setPWM(ch,0,degToPulse(home)); delay(500);
        if (enc>=0) { Serial.print(F("  @home act=")); Serial.println(readActualDeg(enc),1); }
        Serial.println(F("  done"));
      } else Serial.println(F("Use: T ch (0-6)"));
      break;
    }

    case '?': printStartupInfo(); break;

    default: Serial.println(F("H/M/D/F/J/G/S/P/C/E/R/T/?"));
  }
}