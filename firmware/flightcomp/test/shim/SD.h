/**
 * @file SD.h
 * @brief Host stand-in for the Teensy SD library: the File type that board.h and SD_Card.h name,
 * and an in-memory card for the suites that compile SD.cpp.
 *
 * Writes are discarded. A file opened from the card reads its text back one character at a
 * time and returns -1 at the end, as the Teensy SD library's File::read() does.
 *
 * A reader that ignores the end of the file would spin forever on a board. Here each read past
 * the end is counted, and after shimReadsPastEndLimit of them read() throws ShimReadPastEnd, so
 * a test of such a reader fails instead of hanging.
 */
#pragma once

#include <stddef.h>

#include <map>
#include <memory>
#include <string>

/** Thrown by File::read() after shimReadsPastEndLimit reads past the end of a file. */
struct ShimReadPastEnd
{
  long reads;
};
inline long shimReadsPastEndLimit = 100000;

struct File
{
  std::shared_ptr<const std::string> content;
  size_t posn = 0;
  long readsPastEnd = 0;

  int read()
  {
    if (content && posn < content->size())
    {
      return (unsigned char)(*content)[posn++];
    }
    if (++readsPastEnd > shimReadsPastEndLimit)
    {
      throw ShimReadPastEnd{readsPastEnd};
    }
    return -1;
  }
  size_t write(const char *, size_t length) { return length; }
  template <typename... Args>
  size_t print(Args &&...) { return 0; }
  template <typename... Args>
  size_t println(Args &&...) { return 0; }
  void flush() {}
  void close() {}
  explicit operator bool() const { return true; }
};

#define FILE_READ 0
#define FILE_WRITE 1

/** The card: file names and their text. */
struct SDClass
{
  std::map<std::string, std::string> files;

  bool begin(int = 0) { return true; }
  bool exists(const char *name) { return files.count(name) != 0; }
  File open(const char *name, int = FILE_READ)
  {
    File f;
    auto it = files.find(name);
    if (it != files.end())
    {
      f.content = std::make_shared<const std::string>(it->second);
    }
    return f;
  }
  bool remove(const char *name) { return files.erase(name) != 0; }
};
inline SDClass SD;
