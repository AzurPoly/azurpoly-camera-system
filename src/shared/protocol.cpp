/*
 * Implements the named Windows shared-memory mailbox between the EXE and DLL. A mutex protects complete
 * settings/status messages, while magic, size and version checks prevent incompatible processes from
 * interpreting the same bytes.
 */

#include "protocol.hpp"

namespace azp::standalone {
bool valid_settings(const Settings& s) noexcept {
  return s.enabled <= 1 && std::isfinite(s.tv_exposure) && s.tv_exposure >= kMinimumExposureShiftEv && s.tv_exposure <= kMaximumExposureShiftEv &&
         std::isfinite(s.ir_exposure) && s.ir_exposure >= kMinimumExposureShiftEv && s.ir_exposure <= kMaximumExposureShiftEv &&
         valid_camera_resolution(s.camera_resolution) &&
         valid_render_rate(s.render_rate);
}

/*
 * Creates or opens the simulator-specific mapping and mutex, initializes a new mailbox, and rejects an
 * existing mailbox with incompatible magic, size or protocol. Failed setup releases acquired handles.
 *
 * Mapping and mutex names identify the AZP control channel for this simulator process.
 */
bool Mailbox::open(DWORD pid, bool create) {
  close();
  wchar_t name[128];
  std::swprintf(name, 128, L"Local\\AZPCamera.Control.%lu", pid);
  mutex_ = create ? CreateMutexW(nullptr, FALSE, name) : OpenMutexW(SYNCHRONIZE | MUTEX_MODIFY_STATE, FALSE, name);
  if (!mutex_)
    return false;
  std::swprintf(name, 128, L"Local\\AZPCamera.Data.%lu", pid);
  mapping_ = create ? CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, sizeof(Shared), name)
                    : OpenFileMappingW(FILE_MAP_READ | FILE_MAP_WRITE, FALSE, name);
  const bool fresh = create && GetLastError() != ERROR_ALREADY_EXISTS;
  if (!mapping_) {
    close();
    return false;
  }
  memory_ = static_cast<Shared*>(MapViewOfFile(mapping_, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, sizeof(Shared)));
  if (!memory_ || !lock(1000)) {
    close();
    return false;
  }
  if (fresh) {
    *memory_ = {};
    memory_->magic = ProtocolMagic;
    memory_->version = ProtocolVersion;
    memory_->bytes = sizeof(Shared);
  }
  const bool valid = memory_->magic == ProtocolMagic && memory_->version == ProtocolVersion && memory_->bytes == sizeof(Shared);
  unlock();
  if (!valid)
    close();
  return valid;
}

/*
 * Acquires the mailbox only when its contents can be trusted. An abandoned mutex signals a possibly
 * interrupted writer, so that message must not be consumed.
 *
 * The previous writer died; don't consume its possibly partial message.
 */
Mailbox::LockResult Mailbox::try_lock(DWORD timeout) noexcept {
  if (!mutex_)
    return LockResult::invalid;
  const auto result = WaitForSingleObject(mutex_, timeout);
  if (result == WAIT_ABANDONED) {
    if (memory_) {
      memory_->settings.enabled = 0;
      memory_->owner_heartbeat = 0;
    }
    ReleaseMutex(mutex_);
    return LockResult::invalid;
  }
  return result == WAIT_OBJECT_0 ? LockResult::acquired : result == WAIT_TIMEOUT ? LockResult::busy : LockResult::invalid;
}

/*
 * Unmaps the shared message and closes both Windows handles. Clearing local handles makes repeated cleanup
 * safe.
 */
void Mailbox::close() noexcept {
  if (memory_)
    UnmapViewOfFile(memory_);
  if (mapping_)
    CloseHandle(mapping_);
  if (mutex_)
    CloseHandle(mutex_);
  memory_ = nullptr;
  mapping_ = mutex_ = nullptr;
}

}  /* namespace azp::standalone */
