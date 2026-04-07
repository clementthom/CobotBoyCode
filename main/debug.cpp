// ======================================================
// OUTILS DEBUG
// ======================================================

/*
void printLine() {
  Serial.println("----------------------------------------");
}

void printPoint(const char* label, const Coordinates& p) {
  Serial.print(label);
  Serial.print(" : ");
  Serial.print(p.x);
  Serial.print(", ");
  Serial.print(p.y);
  Serial.print(", ");
  Serial.println(p.z);
}

void printIKDebug(const DebugIKData& d) {
  printLine();

  Serial.print("Cible outil : x=");
  Serial.print(d.x);
  Serial.print(" y=");
  Serial.print(d.y);
  Serial.print(" z=");
  Serial.println(d.z);

  Serial.print("rTool = ");
  Serial.print(d.rTool);
  Serial.print(" | rWrist = ");
  Serial.print(d.rWrist);
  Serial.print(" | zWrist = ");
  Serial.println(d.zWrist);

  Serial.print("rp = ");
  Serial.print(d.rp);
  Serial.print(" | zp = ");
  Serial.print(d.zp);
  Serial.print(" | D = ");
  Serial.println(d.D);

  Serial.print("Angles geo -> A: ");
  Serial.print(d.degA);
  Serial.print(" | B: ");
  Serial.print(d.degB);
  Serial.print(" | Z: ");
  Serial.println(d.degZ);

  Serial.print("Cmd servo  -> A: ");
  Serial.print(d.cmdA);
  Serial.print(" | B: ");
  Serial.print(d.cmdB);
  Serial.print(" | Z: ");
  Serial.println(d.cmdZ);

  Serial.print("Reachable  -> ");
  Serial.println(d.reachable ? "OUI" : "NON");
}
*/