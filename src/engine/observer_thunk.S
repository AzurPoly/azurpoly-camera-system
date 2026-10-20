/*
 * Bridges native x64 engine calls to the C++ observers. Each thunk preserves registers, flags and
 * stack/unwind conventions, invokes the relevant observer, then transfers control to the original engine
 * function exactly once.
 */

.text
.p2align 4
.globl azp_engine_hook_thunk
/*
 * Calls the manager observer before tail-jumping to the original manager update. Volatile integer/SIMD
 * registers, flags and MXCSR are preserved. The stack is 16-byte aligned with 32 bytes of observer shadow
 * space; the frame pointer and unwind directives also cover the temporary flag restore before the
 * LEA/POP/JMP epilogue.
 */
.seh_proc azp_engine_hook_thunk
azp_engine_hook_thunk:
  pushq %rbp
  .seh_pushreg %rbp
.globl azp_hook_after_push_rbp
azp_hook_after_push_rbp:
  pushfq
  .seh_stackalloc 8
.globl azp_hook_after_push_flags
azp_hook_after_push_flags:
  subq $216, %rsp
  .seh_stackalloc 216
.globl azp_hook_after_alloc
azp_hook_after_alloc:
  leaq 224(%rsp), %rbp
  .seh_setframe %rbp, 224
  .seh_endprologue
.globl azp_hook_after_frame
azp_hook_after_frame:

  movq %rax, 32(%rsp)
  movq %rcx, 40(%rsp)
  movq %rdx, 48(%rsp)
  movq %r8, 56(%rsp)
  movq %r9, 64(%rsp)
  movq %r10, 72(%rsp)
  movq %r11, 80(%rsp)
  movdqu %xmm0, 96(%rsp)
  movdqu %xmm1, 112(%rsp)
  movdqu %xmm2, 128(%rsp)
  movdqu %xmm3, 144(%rsp)
  movdqu %xmm4, 160(%rsp)
  movdqu %xmm5, 176(%rsp)
  stmxcsr 192(%rsp)
  call azp_engine_hook_observe
  ldmxcsr 192(%rsp)
  movdqu 96(%rsp), %xmm0
  movdqu 112(%rsp), %xmm1
  movdqu 128(%rsp), %xmm2
  movdqu 144(%rsp), %xmm3
  movdqu 160(%rsp), %xmm4
  movdqu 176(%rsp), %xmm5
  movq 32(%rsp), %rax
  movq 40(%rsp), %rcx
  movq 48(%rsp), %rdx
  movq 56(%rsp), %r8
  movq 64(%rsp), %r9
  movq 72(%rsp), %r10
  movq 80(%rsp), %r11

.globl azp_hook_before_restore_flags
azp_hook_before_restore_flags:
  pushq -8(%rbp)
.globl azp_hook_after_restore_push
azp_hook_after_restore_push:
  popfq
.globl azp_hook_after_restore_flags
azp_hook_after_restore_flags:
  leaq (%rbp), %rsp
.globl azp_hook_after_stack_restore
azp_hook_after_stack_restore:
  popq %rbp
.globl azp_hook_after_frame_restore
azp_hook_after_frame_restore:
  jmpq *azp_engine_hook_original(%rip)
.seh_endproc


.text
.p2align 4
.globl azp_camera_pose_hook_thunk
/*
 * Calls the live camera-pose observer before tail-jumping to the original node calculation. Volatile
 * integer/SIMD registers, flags and MXCSR are preserved. The stack is 16-byte aligned with 32 bytes of
 * observer shadow space; the frame pointer and unwind directives also cover the temporary flag restore
 * before the LEA/POP/JMP epilogue.
 */
.seh_proc azp_camera_pose_hook_thunk
azp_camera_pose_hook_thunk:
  pushq %rbp
  .seh_pushreg %rbp
.globl azp_camera_pose_after_push_rbp
azp_camera_pose_after_push_rbp:
  pushfq
  .seh_stackalloc 8
.globl azp_camera_pose_after_push_flags
azp_camera_pose_after_push_flags:
  subq $216, %rsp
  .seh_stackalloc 216
.globl azp_camera_pose_after_alloc
azp_camera_pose_after_alloc:
  leaq 224(%rsp), %rbp
  .seh_setframe %rbp, 224
  .seh_endprologue
.globl azp_camera_pose_after_frame
azp_camera_pose_after_frame:

  movq %rax, 32(%rsp)
  movq %rcx, 40(%rsp)
  movq %rdx, 48(%rsp)
  movq %r8, 56(%rsp)
  movq %r9, 64(%rsp)
  movq %r10, 72(%rsp)
  movq %r11, 80(%rsp)
  movdqu %xmm0, 96(%rsp)
  movdqu %xmm1, 112(%rsp)
  movdqu %xmm2, 128(%rsp)
  movdqu %xmm3, 144(%rsp)
  movdqu %xmm4, 160(%rsp)
  movdqu %xmm5, 176(%rsp)
  stmxcsr 192(%rsp)
  call azp_camera_pose_hook_observe
  ldmxcsr 192(%rsp)
  movdqu 96(%rsp), %xmm0
  movdqu 112(%rsp), %xmm1
  movdqu 128(%rsp), %xmm2
  movdqu 144(%rsp), %xmm3
  movdqu 160(%rsp), %xmm4
  movdqu 176(%rsp), %xmm5
  movq 32(%rsp), %rax
  movq 40(%rsp), %rcx
  movq 48(%rsp), %rdx
  movq 56(%rsp), %r8
  movq 64(%rsp), %r9
  movq 72(%rsp), %r10
  movq 80(%rsp), %r11

.globl azp_camera_pose_before_restore_flags
azp_camera_pose_before_restore_flags:
  pushq -8(%rbp)
.globl azp_camera_pose_after_restore_push
azp_camera_pose_after_restore_push:
  popfq
.globl azp_camera_pose_after_restore_flags
azp_camera_pose_after_restore_flags:
  leaq (%rbp), %rsp
.globl azp_camera_pose_after_stack_restore
azp_camera_pose_after_stack_restore:
  popq %rbp
.globl azp_camera_pose_after_frame_restore
azp_camera_pose_after_frame_restore:
  jmpq *azp_camera_pose_hook_original(%rip)
.seh_endproc
