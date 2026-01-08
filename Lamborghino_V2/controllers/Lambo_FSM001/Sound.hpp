#ifndef SOUND_HPP
#define SOUND_HPP

#include <webots/Supervisor.hpp>      // Header of Webots Supervisor
#include <webots/Speaker.hpp>

class Sound {
public:
  Sound(webots::Supervisor *robot);        // Module to initialize speaker
  void play(int track);                    // Function to play sounds

private:
  webots::Speaker *buzzer;
};

#endif
