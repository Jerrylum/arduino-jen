#pragma once

// https://github.com/josephlarralde/ArduinoEventEmitter

#ifndef _ARDUINO_EVENT_EMITTER_H_
#define _ARDUINO_EVENT_EMITTER_H_

#include "jen.hpp"

#define EVENT_EMITTER_MAX_LISTENERS 12
#define EVENT_EMITTER_MAX_EVENT_STRING_SIZE 32

template <class... T>
class EventEmitter {
 private:
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  class Listener {
   private:
    char eventName[EVENT_EMITTER_MAX_EVENT_STRING_SIZE];
    void (*callback)(T... t);

   public:
    Listener(const char *name, void (*cb)(T... t)) : callback(cb) { strcpy(eventName, name); }

    ~Listener() {}

    const char *getEventName() { return static_cast<const char *>(eventName); }

    void emit(T... t) { callback(t...); }
  };



 public:
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  unsigned int nbListeners;
  Listener *listeners[EVENT_EMITTER_MAX_LISTENERS];

  EventEmitter() : nbListeners(0) {
    for (unsigned int i = 0; i < EVENT_EMITTER_MAX_LISTENERS; ++i) {
      listeners[i] = NULL;
    }
  }

  virtual ~EventEmitter() {}

  bool addListener(const char *name, void (*cb)(T... t)) {
    if (nbListeners < EVENT_EMITTER_MAX_LISTENERS) {
      for (unsigned int i = 0; i < EVENT_EMITTER_MAX_LISTENERS; ++i) {
        if (listeners[i] == NULL) {
          listeners[i] = new Listener(name, cb);
          nbListeners++;
          return true;
        }
      }
    }

    return false;
  }

  bool removeListeners(const char *name) {
    bool removed = false;
    for (unsigned int i = 0; i < EVENT_EMITTER_MAX_LISTENERS; ++i) {
      if (listeners[i] != NULL && strcmp(name, listeners[i]->getEventName()) == 0) {
        delete listeners[i];
        listeners[i] = NULL;
        nbListeners--;
        removed = true;
      }
    }

    return removed;
  }

  void removeAllListeners() {
    for (unsigned int i = 0; i < EVENT_EMITTER_MAX_LISTENERS; ++i) {
      if (listeners[i] != NULL) {
        delete listeners[i];
        nbListeners = 0;
      }
    }
  }

  void emit(const char *name, T... t) {
    for (unsigned int i = 0; i < EVENT_EMITTER_MAX_LISTENERS; ++i) {
      if (listeners[i] != NULL && strcmp(name, listeners[i]->getEventName()) == 0) {
        listeners[i]->emit(t...);
      }
    }
  }
};

#endif /* _ARDUINO_EVENT_EMITTER_H_ */
