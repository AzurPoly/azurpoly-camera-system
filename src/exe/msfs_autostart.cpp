/*
 * Reads and edits exe.xml as text. MSFS reads the file at startup and other add-ons keep their own entries in
 * it, so the edit is deliberately narrow: entries are found outside comments and CDATA sections, AZP Camera's
 * are cut out whole, and the new one goes just before </SimBase.Document>. The previous file is kept as
 * exe.xml.azp-camera.bak and the new one replaces it in a single rename.
 */

#include "msfs_autostart.hpp"
#include "launcher.hpp"
#include "resources.hpp"
#include <cwctype>
#include "../shared/configuration.hpp"

namespace azp::app {
namespace {
constexpr const wchar_t* kEntryName = L"" AZP_CAMERA_PRODUCT_NAME;
constexpr const wchar_t* kExecutableName = L"AZP-Camera.exe";
constexpr const wchar_t* kArguments = L"--silent";
constexpr DWORD kMaximumFileBytes = 1024 * 1024;
constexpr auto npos = std::wstring::npos;

struct Installation {
  const wchar_t* edition;
  std::wstring folder;
};

std::wstring environment(const wchar_t* name) {
  wchar_t value[32768]{};
  const DWORD length = GetEnvironmentVariableW(name, value, 32768);
  return length && length < 32768 ? std::wstring(value, length) : std::wstring();
}

/*
 * MSFS 2024 installations whose settings folder exists; the simulator creates it when it first starts.
 */
std::vector<Installation> installations() {
  const auto roaming = environment(L"APPDATA"), local = environment(L"LOCALAPPDATA");
  const Installation candidates[] = {
      {L"Steam", roaming.empty() ? std::wstring() : roaming + kSteamSettingsFolder},
      {L"Microsoft Store", local.empty() ? std::wstring() : local + kStoreSettingsFolder},
  };
  std::vector<Installation> found;
  for (const auto& candidate : candidates) {
    const DWORD attributes = candidate.folder.empty() ? INVALID_FILE_ATTRIBUTES : GetFileAttributesW(candidate.folder.c_str());
    if (attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY))
      found.push_back(candidate);
  }
  return found;
}

enum class Encoding { utf8, utf8_bom, windows1252 };

/*
 * exe.xml as text. `exists` is false for a file still to be created.
 */
struct Document {
  bool exists = false;
  Encoding encoding = Encoding::windows1252;
  std::wstring text;
};

std::string lowercase(std::string text) {
  for (auto& c : text)
    if (c >= 'A' && c <= 'Z')
      c = static_cast<char>(c - 'A' + 'a');
  return text;
}

/*
 * The encoding named by the XML declaration; without a declaration XML is UTF-8. MSFS writes Windows-1252.
 * False for any other encoding: such a file is left alone rather than rewritten.
 */
bool declared_encoding(const std::string& bytes, Encoding& encoding) {
  encoding = Encoding::utf8;
  if (bytes.compare(0, 5, "<?xml") != 0)
    return true;
  const auto end = bytes.find("?>");
  const auto name = bytes.find("encoding");
  if (end == std::string::npos || name == std::string::npos || name > end)
    return true;
  const auto quote = bytes.find_first_of("\"'", name);
  const auto close = quote == std::string::npos ? std::string::npos : bytes.find(bytes[quote], quote + 1);
  if (close == std::string::npos || close > end)
    return false;
  const auto value = lowercase(bytes.substr(quote + 1, close - quote - 1));
  if (value == "windows-1252" || value == "cp1252" || value == "iso-8859-1" || value == "us-ascii")
    encoding = Encoding::windows1252;
  else if (value != "utf-8" && value != "utf8")
    return false;
  return true;
}

bool decode(const std::string& bytes, UINT code_page, std::wstring& text) {
  text.clear();
  if (bytes.empty())
    return true;
  const DWORD flags = code_page == CP_UTF8 ? MB_ERR_INVALID_CHARS : 0;
  const int size = MultiByteToWideChar(code_page, flags, bytes.data(), static_cast<int>(bytes.size()), nullptr, 0);
  if (size <= 0)
    return false;
  text.assign(size, L'\0');
  return MultiByteToWideChar(code_page, flags, bytes.data(), static_cast<int>(bytes.size()), text.data(), size) == size;
}

/*
 * False when a character has no exact equivalent in `code_page`.
 */
bool encode(const std::wstring& text, UINT code_page, std::string& bytes) {
  bytes.clear();
  if (text.empty())
    return true;
  const bool utf8 = code_page == CP_UTF8;
  BOOL lossy = FALSE;
  const DWORD flags = utf8 ? WC_ERR_INVALID_CHARS : WC_NO_BEST_FIT_CHARS;
  const int size =
      WideCharToMultiByte(code_page, flags, text.data(), static_cast<int>(text.size()), nullptr, 0, nullptr, utf8 ? nullptr : &lossy);
  if (size <= 0 || lossy)
    return false;
  bytes.assign(size, '\0');
  return WideCharToMultiByte(code_page, flags, text.data(), static_cast<int>(text.size()), bytes.data(), size, nullptr,
                             utf8 ? nullptr : &lossy) == size &&
         !lossy;
}

bool read_document(const std::wstring& path, Document& document, std::string& error) {
  document = {};
  HANDLE file = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING,
                            FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    const DWORD code = GetLastError();
    if (code == ERROR_FILE_NOT_FOUND)
      return true;
    error = "exe.xml cannot be read (Windows error " + std::to_string(code) + ").";
    return false;
  }
  std::string bytes(kMaximumFileBytes, '\0');
  DWORD read = 0;
  const bool complete = ReadFile(file, bytes.data(), kMaximumFileBytes, &read, nullptr) && read < kMaximumFileBytes;
  CloseHandle(file);
  if (!complete) {
    error = "exe.xml cannot be read.";
    return false;
  }
  bytes.resize(read);
  document.exists = true;
  const auto byte = [&](std::size_t at) { return at < bytes.size() ? static_cast<unsigned char>(bytes[at]) : 0u; };
  if ((byte(0) == 0xFF && byte(1) == 0xFE) || (byte(0) == 0xFE && byte(1) == 0xFF)) {
    error = "exe.xml is UTF-16: left unchanged.";
    return false;
  }
  if (byte(0) == 0xEF && byte(1) == 0xBB && byte(2) == 0xBF) {
    bytes.erase(0, 3);
    document.encoding = Encoding::utf8_bom;
  } else if (!declared_encoding(bytes, document.encoding)) {
    error = "exe.xml declares an unsupported encoding: left unchanged.";
    return false;
  }
  if (!decode(bytes, document.encoding == Encoding::windows1252 ? 1252 : CP_UTF8, document.text)) {
    error = "exe.xml is not valid text in its declared encoding.";
    return false;
  }
  return true;
}

/*
 * Writes the new file beside exe.xml, keeps the previous one as exe.xml.azp-camera.bak, then renames the new
 * file over exe.xml.
 */
bool write_document(const std::wstring& path, const Document& document, std::string& error) {
  std::string bytes;
  if (!encode(document.text, document.encoding == Encoding::windows1252 ? 1252 : CP_UTF8, bytes)) {
    error = "The folder of AZP-Camera.exe has characters exe.xml (Windows-1252) cannot store. Move it to another folder.";
    return false;
  }
  if (document.encoding == Encoding::utf8_bom)
    bytes.insert(0, "\xEF\xBB\xBF");
  const auto temporary = path + L".azp-camera.tmp";
  HANDLE file = CreateFileW(temporary.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    error = "exe.xml cannot be written (Windows error " + std::to_string(GetLastError()) + ").";
    return false;
  }
  DWORD written = 0;
  const bool complete =
      WriteFile(file, bytes.data(), static_cast<DWORD>(bytes.size()), &written, nullptr) && written == bytes.size() && FlushFileBuffers(file);
  CloseHandle(file);
  const bool saved = complete && (!document.exists || CopyFileW(path.c_str(), (path + L".azp-camera.bak").c_str(), FALSE)) &&
                     MoveFileExW(temporary.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH);
  if (!saved) {
    error = "exe.xml cannot be written (Windows error " + std::to_string(GetLastError()) + ").";
    DeleteFileW(temporary.c_str());
  }
  return saved;
}

/*
 * Position of the tag `tag` (for example L"<Launch.Addon") at or after `from`, outside comments and CDATA
 * sections; npos when there is none.
 */
std::size_t find_tag(const std::wstring& text, const std::wstring& tag, std::size_t from) {
  for (auto at = text.find(L'<', from); at != npos; at = text.find(L'<', at + 1)) {
    if (text.compare(at, 4, L"<!--") == 0 || text.compare(at, 9, L"<![CDATA[") == 0) {
      at = text.find(text[at + 2] == L'-' ? L"-->" : L"]]>", at);
      if (at == npos)
        return npos;
      continue;
    }
    const auto next = at + tag.size();
    if (text.compare(at, tag.size(), tag) == 0 && next < text.size() &&
        (text[next] == L'>' || text[next] == L'/' || std::iswspace(text[next])))
      return at;
  }
  return npos;
}

std::wstring trim(const std::wstring& text) {
  const auto first = text.find_first_not_of(L" \t\r\n");
  return first == npos ? std::wstring() : text.substr(first, text.find_last_not_of(L" \t\r\n") - first + 1);
}

std::wstring unescape(const std::wstring& text) {
  static const std::pair<std::wstring, wchar_t> entities[] = {
      {L"&lt;", L'<'}, {L"&gt;", L'>'}, {L"&quot;", L'"'}, {L"&apos;", L'\''}, {L"&amp;", L'&'}};
  std::wstring result;
  for (std::size_t at = 0; at < text.size(); ++at) {
    bool replaced = false;
    for (const auto& [entity, character] : entities)
      if (!replaced && text.compare(at, entity.size(), entity) == 0) {
        result += character;
        at += entity.size() - 1;
        replaced = true;
      }
    if (!replaced)
      result += text[at];
  }
  return result;
}

std::wstring escape(const std::wstring& text) {
  std::wstring result;
  for (const wchar_t c : text)
    result += c == L'&' ? L"&amp;" : c == L'<' ? L"&lt;" : c == L'>' ? L"&gt;" : std::wstring(1, c);
  return result;
}

std::wstring unquoted(const std::wstring& path) {
  return path.size() >= 2 && path.front() == L'"' && path.back() == L'"' ? path.substr(1, path.size() - 2) : path;
}

/*
 * One Launch.Addon element: its extent [begin, end) and the values that identify and enable it.
 */
struct Entry {
  std::size_t begin = 0, end = 0;
  std::wstring name, path, arguments, disabled;
};

/*
 * The text of the child element `name` between `begin` and `end`, unescaped and trimmed.
 */
std::wstring child_value(const std::wstring& text, std::size_t begin, std::size_t end, const std::wstring& name) {
  const auto open = find_tag(text, L"<" + name, begin);
  const auto start = open == npos || open >= end ? npos : text.find(L'>', open);
  if (start == npos || start >= end || text[start - 1] == L'/')
    return {};
  const auto close = text.find(L"</" + name, start);
  return close == npos || close > end ? std::wstring() : trim(unescape(text.substr(start + 1, close - start - 1)));
}

/*
 * Every Launch.Addon element. False when one has no end tag: such a file is not edited.
 */
bool find_entries(const std::wstring& text, std::vector<Entry>& entries) {
  entries.clear();
  for (auto at = find_tag(text, L"<Launch.Addon", 0); at != npos;) {
    const auto open_end = text.find(L'>', at);
    if (open_end == npos)
      return false;
    Entry entry;
    entry.begin = at;
    entry.end = open_end + 1;
    if (text[open_end - 1] != L'/') {
      const auto close = find_tag(text, L"</Launch.Addon", open_end);
      const auto close_end = close == npos ? npos : text.find(L'>', close);
      if (close_end == npos)
        return false;
      entry.end = close_end + 1;
      entry.name = child_value(text, open_end, close, L"Name");
      entry.path = child_value(text, open_end, close, L"Path");
      entry.arguments = child_value(text, open_end, close, L"CommandLine");
      entry.disabled = child_value(text, open_end, close, L"Disabled");
    }
    entries.push_back(entry);
    at = find_tag(text, L"<Launch.Addon", entry.end);
  }
  return true;
}

/*
 * AZP Camera's entries: named with the product name, or starting an AZP-Camera.exe.
 */
bool ours(const Entry& entry) {
  if (_wcsicmp(entry.name.c_str(), kEntryName) == 0)
    return true;
  const auto path = unquoted(entry.path);
  const auto slash = path.find_last_of(L"\\/");
  return _wcsicmp((slash == npos ? path : path.substr(slash + 1)).c_str(), kExecutableName) == 0;
}

bool starts(const Entry& entry, const std::wstring& executable) {
  return _wcsicmp(entry.disabled.c_str(), L"True") != 0 && _wcsicmp(entry.arguments.c_str(), kArguments) == 0 &&
         standalone::same_path(unquoted(entry.path), executable);
}

std::wstring new_document(Encoding encoding) {
  return std::wstring(L"<?xml version=\"1.0\" encoding=\"") + (encoding == Encoding::windows1252 ? L"Windows-1252" : L"UTF-8") +
         L"\"?>\r\n"
         L"<SimBase.Document Type=\"SimConnect\" version=\"1,0\">\r\n"
         L"  <Descr>Auto launch external applications on MSFS start</Descr>\r\n"
         L"  <Filename>exe.xml</Filename>\r\n"
         L"  <Disabled>False</Disabled>\r\n"
         L"  <Launch.ManualLoad>False</Launch.ManualLoad>\r\n"
         L"</SimBase.Document>\r\n";
}

/*
 * Removes AZP Camera's entries from `text`, each with its own line when it has one, and, when `executable` is
 * not empty, adds one that starts it just before </SimBase.Document>. False when there is no such end tag.
 */
bool edit(std::wstring& text, const std::vector<Entry>& entries, const std::wstring& executable) {
  const std::wstring eol = text.find(L"\r\n") != npos || text.find(L'\n') == npos ? L"\r\n" : L"\n";
  const auto line_start = [&](std::size_t at) {
    while (at > 0 && (text[at - 1] == L' ' || text[at - 1] == L'\t'))
      --at;
    return at;
  };
  for (auto entry = entries.rbegin(); entry != entries.rend(); ++entry) {
    if (!ours(*entry))
      continue;
    auto begin = entry->begin, end = entry->end;
    const auto line = line_start(begin);
    if (line == 0 || text[line - 1] == L'\n') {
      begin = line;
      auto after = end;
      while (after < text.size() && (text[after] == L' ' || text[after] == L'\t'))
        ++after;
      if (after < text.size() && text[after] == L'\r')
        ++after;
      if (after < text.size() && text[after] == L'\n')
        end = after + 1;
    }
    text.erase(begin, end - begin);
  }
  if (executable.empty())
    return true;
  std::size_t close = npos;
  for (auto at = find_tag(text, L"</SimBase.Document", 0); at != npos; at = find_tag(text, L"</SimBase.Document", at + 1))
    close = at;
  if (close == npos)
    return false;
  const auto entry = L"  <Launch.Addon>" + eol + L"    <Name>" + kEntryName + L"</Name>" + eol + L"    <Disabled>False</Disabled>" + eol +
                     L"    <ManualLoad>False</ManualLoad>" + eol + L"    <Path>" + escape(executable) + L"</Path>" + eol +
                     L"    <CommandLine>" + kArguments + L"</CommandLine>" + eol + L"    <NewConsole>False</NewConsole>" + eol +
                     L"  </Launch.Addon>" + eol;
  const auto line = line_start(close);
  if (line == 0 || text[line - 1] == L'\n')
    text.insert(line, entry);
  else
    text.insert(close, eol + entry);
  return true;
}

SimulatorStartup startup_of(const Installation& installation, const std::wstring& executable) {
  SimulatorStartup startup;
  startup.edition = installation.edition;
  startup.file = installation.folder + L"\\exe.xml";
  Document document;
  std::vector<Entry> entries;
  if (!read_document(startup.file, document, startup.error))
    return startup;
  if (!find_entries(document.text, entries)) {
    startup.error = "exe.xml is incomplete: left unchanged.";
    return startup;
  }
  for (const auto& entry : entries)
    if (ours(entry) && starts(entry, executable))
      startup.current = true;
    else if (ours(entry) && startup.other_path.empty())
      startup.other_path = unquoted(entry.path);
  if (startup.current)
    startup.other_path.clear();
  return startup;
}

/*
 * Brings one installation's exe.xml to the requested state. An entry already starting this EXE, alone, is
 * left as it is.
 */
bool change(const Installation& installation, const std::wstring& executable, bool enabled, std::string& error) {
  const auto path = installation.folder + L"\\exe.xml";
  Document document;
  std::vector<Entry> entries;
  if (!read_document(path, document, error))
    return false;
  if (!document.exists) {
    if (!enabled)
      return true;
    std::string probe;
    document.encoding = encode(executable, 1252, probe) ? Encoding::windows1252 : Encoding::utf8;
    document.text = new_document(document.encoding);
  }
  if (!find_entries(document.text, entries)) {
    error = "exe.xml is incomplete: left unchanged.";
    return false;
  }
  std::size_t own = 0;
  bool starting = false;
  for (const auto& entry : entries)
    if (ours(entry)) {
      ++own;
      starting = starting || starts(entry, executable);
    }
  if (enabled ? own == 1 && starting : own == 0)
    return true;
  if (!edit(document.text, entries, enabled ? executable : std::wstring())) {
    error = "exe.xml has no </SimBase.Document> end tag: left unchanged.";
    return false;
  }
  return write_document(path, document, error);
}
}  /* namespace */

std::vector<SimulatorStartup> simulator_startup(const std::wstring& executable) {
  std::vector<SimulatorStartup> result;
  for (const auto& installation : installations())
    result.push_back(startup_of(installation, executable));
  return result;
}

std::vector<SimulatorStartup> set_simulator_startup(const std::wstring& executable, bool enabled, bool& ok) {
  const auto found = installations();
  std::vector<SimulatorStartup> result;
  ok = !found.empty();
  for (const auto& installation : found) {
    std::string error;
    const bool changed = change(installation, executable, enabled, error);
    result.push_back(startup_of(installation, executable));
    if (!changed) {
      ok = false;
      if (result.back().error.empty())
        result.back().error = error;
    }
  }
  return result;
}

}  /* namespace azp::app */
