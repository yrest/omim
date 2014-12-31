#pragma once
#include "file_reader.hpp"
#include "file_writer.hpp"

#include "../std/vector.hpp"
#include "../std/string.hpp"
#include "../std/noncopyable.hpp"


class FilesContainerBase
{
public:
  typedef string Tag;

  bool IsExist(Tag const & tag) const
  {
    return GetInfo(tag) != 0;
  }

protected:
  struct Info
  {
    Tag m_tag;
    uint64_t m_offset;
    uint64_t m_size;

    Info() {}
    Info(Tag const & tag, uint64_t offset) : m_tag(tag), m_offset(offset) {}
  };

  friend string DebugPrint(Info const & info);

  Info const * GetInfo(Tag const & tag) const;

  struct LessInfo
  {
    bool operator() (Info const & t1, Info const & t2) const
    {
      return (t1.m_tag < t2.m_tag);
    }
    bool operator() (Info const & t1, Tag const & t2) const
    {
      return (t1.m_tag < t2);
    }
    bool operator() (Tag const & t1, Info const & t2) const
    {
      return (t1 < t2.m_tag);
    }
  };

  struct LessOffset
  {
    bool operator() (Info const & t1, Info const & t2) const
    {
      if (t1.m_offset == t2.m_offset)
      {
        // Element with nonzero size should be the last one,
        // for correct append writer mode (FilesContainerW::GetWriter).
        return (t1.m_size < t2.m_size);
      }
      else
        return (t1.m_offset < t2.m_offset);
    }
    bool operator() (Info const & t1, uint64_t const & t2) const
    {
      return (t1.m_offset < t2);
    }
    bool operator() (uint64_t const & t1, Info const & t2) const
    {
      return (t1 < t2.m_offset);
    }
  };

  class EqualTag
  {
    Tag const & m_tag;
  public:
    EqualTag(Tag const & tag) : m_tag(tag) {}
    bool operator() (Info const & t) const
    {
      return (t.m_tag == m_tag);
    }
  };

  typedef vector<Info> InfoContainer;
  InfoContainer m_info;

  template <class ReaderT>
  void ReadInfo(ReaderT & reader);

public:
  template <class ToDo> void ForEachTag(ToDo toDo) const
  {
    for_each(m_info.begin(), m_info.end(), toDo);
  }
};

class FilesContainerR : public FilesContainerBase
{
public:
  typedef ModelReaderPtr ReaderT;

  explicit FilesContainerR(string const & fName,
                           uint32_t logPageSize = 10,
                           uint32_t logPageCount = 10);
  explicit FilesContainerR(ReaderT const & file);

  ReaderT GetReader(Tag const & tag) const;

  template <typename F> void ForEachTag(F f) const
  {
    for (size_t i = 0; i < m_info.size(); ++i)
      f(m_info[i].m_tag);
  }

  inline uint64_t GetFileSize() const { return m_source.Size(); }
  inline string const & GetFileName() const { return m_source.GetName(); }

private:
  ReaderT m_source;
};

class FilesMappingContainer : public FilesContainerBase
{
public:
  FilesMappingContainer();
  explicit FilesMappingContainer(string const & fName);

  ~FilesMappingContainer();

  void Open(string const & fName);
  void Close();

  class Handle : private noncopyable
  {
    void Reset();

  public:
    Handle()
      : m_base(0), m_origBase(0), m_size(0), m_origSize(0)
    {
    }
    Handle(char const * base, char const * alignBase, uint64_t size, uint64_t origSize)
      : m_base(base), m_origBase(alignBase), m_size(size), m_origSize(origSize)
    {
    }
    Handle(Handle && h)
    {
      Assign(std::move(h));
    }
    ~Handle();

    void Assign(Handle && h);

    void Unmap();

    bool IsValid() const { return (m_base != 0); }
    uint64_t GetSize() const { return m_size; }

    template <class T> T const * GetData() const
    {
      ASSERT_EQUAL(m_size % sizeof(T), 0, ());
      return reinterpret_cast<T const *>(m_base);
    }
    template <class T> size_t GetDataCount() const
    {
      ASSERT_EQUAL(m_size % sizeof(T), 0, ());
      return (m_size / sizeof(T));
    }

  private:
    char const * m_base;
    char const * m_origBase;
    uint64_t m_size;
    uint64_t m_origSize;
  };

  Handle Map(Tag const & tag) const;
  FileReader GetReader(Tag const & tag) const;

  string const & GetName() const { return m_name; }

private:
  string m_name;
  #ifdef OMIM_OS_WINDOWS
    void * m_hFile;
    void * m_hMapping;
  #else
    int m_fd;
  #endif
};

class FilesContainerW : public FilesContainerBase
{
public:
  FilesContainerW(string const & fName,
                  FileWriter::Op op = FileWriter::OP_WRITE_TRUNCATE);
  ~FilesContainerW();

  FileWriter GetWriter(Tag const & tag);

  void Write(string const & fPath, Tag const & tag);
  void Write(ModelReaderPtr reader, Tag const & tag);
  void Write(vector<char> const & buffer, Tag const & tag);

  void Finish();

  /// Delete section with rewriting file.
  /// @precondition Container should be opened with FileWriter::OP_WRITE_EXISTING.
  void DeleteSection(Tag const & tag);

  inline string const & GetFileName() const { return m_name; }

private:
  uint64_t SaveCurrentSize();

  void Open(FileWriter::Op op);
  void StartNew();

  string m_name;
  bool m_bNeedRewrite;
  bool m_bFinished;
};
