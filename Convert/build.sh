#!/usr/bin/env bash
# ============================================================================
# build.sh — сборка конвертатора STL -> MDL (Source Engine 2007)
#   Linux ELF  : build/stl2mdl            (gcc, всегда)
#   Windows PE : build/stl2mdl.exe        (x86_64-w64-mingw32-gcc, если есть)
#                build/stl2mdl.x86.exe    (i686-w64-mingw32-gcc, если есть)
#
# Ускорение: -O3 -march=native -flto + многопоточная компиляция (jobs).
# Использование: ./build.sh [all|linux|win] [-jN] [--no-lto] [--portable]
# ============================================================================
set -e
cd "$(dirname "$0")"

SRC_DIR="src"
OUT_DIR="build"
BIN="stl2mdl"
SRCS=( "$SRC_DIR/main.c" "$SRC_DIR/stl.c" "$SRC_DIR/buf.c"
       "$SRC_DIR/simplify.c" "$SRC_DIR/physics.c" "$SRC_DIR/mdlwrite.c"
       "$SRC_DIR/vtx.c" "$SRC_DIR/xmem.c" )

MODE="all"
JOBS="$( (nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4) )"
LTO=1
PORTABLE=0

for a in "$@"; do
  case "$a" in
    all|linux|win)      MODE="$a" ;;
    -j*)                JOBS="${a#-j}" ;;
    --no-lto)           LTO=0 ;;
    --portable)         PORTABLE=1 ;;
    -h|--help)
      echo "usage: $0 [all|linux|win] [-jN] [--no-lto] [--portable]"; exit 0 ;;
    *) echo "unknown option: $a" >&2; exit 1 ;;
  esac
done

mkdir -p "$OUT_DIR"

CFLAGS_BASE="-std=c11 -O3 -Wall -Wextra -Wno-unused-parameter -D_GNU_SOURCE -I$SRC_DIR"
[ "$LTO" = 1 ] && CFLAGS_BASE+=" -flto"
if [ "$PORTABLE" = 0 ]; then
  # -march=native ускоряет геометрию/квантование на x86 (SSE/AVX авто-векторизация).
  # Для сборки "на вынос" на другой CPU используйте --portable.
  if gcc -march=native -x c - <<< 'int main(void){return 0;}' -o /tmp/.arch_probe 2>/dev/null; then
    CFLAGS_BASE+=" -march=native"
  fi
  rm -f /tmp/.arch_probe 2>/dev/null || true
fi
LDFLAGS="-lm"
[ "$LTO" = 1 ] && LDFLAGS+=" -flto"

pick_cc() {  # $1 = candidate
  command -v "$1" >/dev/null 2>&1 && echo "$1" && return 0
  return 1
}

build_one() {  # $1=compiler  $2=output
  local CC="$1" OUT="$2" objlist=()
  echo ">> [$CC] -> $OUT  (-j$JOBS)"
  # Параллельная компиляция .c -> .o (ускорение сборки на многоядерных машинах)
  local pids=() objs=() srcs_i=0
  for src in "${SRCS[@]}"; do
    local obj="$OUT_DIR/$(basename "${src%.c}").$(basename "$CC" | tr -dc 'a-z0-9').o"
    ( $CC $CFLAGS_BASE -c "$src" -o "$obj" ) &
    pids+=($!); objs+=("$obj")
    # ограничиваем число параллельных задач
    while [ "$(jobs -rp | wc -l)" -ge "$JOBS" ]; do wait -n 2>/dev/null || break; done
  done
  local fail=0
  for p in "${pids[@]}"; do wait "$p" || fail=1; done
  [ "$fail" = 1 ] && { echo "!! compile failed ($CC)" >&2; return 1; }
  $CC $CFLAGS_BASE "${objs[@]}" -o "$OUT" $LDFLAGS
  echo "   OK: $OUT ($(stat -c%s "$OUT" 2>/dev/null || stat -f%z "$OUT") bytes)"
}

case "$MODE" in
  linux|all)
    CC_LINUX="$(pick_cc gcc || pick_cc clang || echo cc)"
    build_one "$CC_LINUX" "$OUT_DIR/$BIN"
    ;;
esac

case "$MODE" in
  win|all)
    BUILT_WIN=0
    if CC_MW64="$(pick_cc x86_64-w64-mingw32-gcc)"; then
      build_one "$CC_MW64" "$OUT_DIR/$BIN.exe"; BUILT_WIN=1
    fi
    if CC_MW32="$(pick_cc i686-w64-mingw32-gcc)"; then
      build_one "$CC_MW32" "$OUT_DIR/$BIN.x86.exe"; BUILT_WIN=1
    fi
    if [ "$BUILT_WIN" = 0 ]; then
      echo "-- mingw-w64 не найден — пропускаем сборку .exe."
      echo "   Debian/Ubuntu: sudo apt install mingw-w64"
      echo "   Fedora:        sudo dnf install mingw64-gcc"
      echo "   Arch:          sudo pacman -S mingw-w64-gcc"
      echo "   Или запустите этот скрипт с Windows (MinGW/msys2): ./build.sh win"
      [ "$MODE" = "win" ] && exit 3
    fi
    ;;
esac

echo "== Готово =="
ls -la "$OUT_DIR"/$BIN* 2>/dev/null || true
