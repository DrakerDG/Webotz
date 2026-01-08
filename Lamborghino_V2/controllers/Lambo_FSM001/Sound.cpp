#include "Sound.hpp"

using namespace webots;

Sound::Sound(Supervisor *robot) {             // Module to initialize speaker (buzzer)
  buzzer = robot->getSpeaker("buzzer");
}

void Sound::play(int track) {                // Function to play sounds
  if (track == 0)
    buzzer->playSound(buzzer, buzzer, "sounds/beep.wav", 1, 1, 0, false);
  else if (track == 1)
    buzzer->playSound(buzzer, buzzer, "sounds/start.wav", 1, 1, 0, false);
  else if (track == 2)
    buzzer->playSound(buzzer, buzzer, "sounds/goal.wav", 1, 1, 0, false);
}
