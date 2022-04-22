#ifndef PRINT_UTILS_H
#define PRINT_UTILS_H

int printFloat(float value, char* buffer) {
  value = max(-9.9, value);
  value = min(99.9, value);
  dtostrf(value, 4, 1, buffer);
}

#endif
