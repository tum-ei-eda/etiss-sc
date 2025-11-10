#!/usr/bin/env bash
########################################################################################################################
# Package dependencies
get_apt_deps() {
  etisssc_apt_dep="cmake build-essential python3 python3-virtualenv python3-dev"
  etiss_apt_dep="libboost-filesystem-dev libboost-system-dev libboost-program-options-dev"
  echo "${etisssc_apt_dep} ${etiss_apt_dep}"
}
setup_env() {
  apt-get update
  for pkg in "$(get_apt_deps)"
  do 
    apt-get install --no-install-recommends -y ${pkg}
  done

}
########################################################################################################################
# Python virtual env
setup_pyvenv() {
  venv_dir="$1"
  req_file="$2"

  echo "[setup] venv"
  python3 -m virtualenv "${venv_dir}"
  . "${venv_dir}/bin/activate"
  pip install --upgrade pip
  if [ -f "${req_file}" ]; then
    pip install -r "${req_file}"
  fi
}
########################################################################################################################
# SystemC
fetch_systemc() {
  src_dir="$1"
  build_dir="$2"
  install_dir="$3"
  version="${4}"

  systemc_tag="${version}"
  systemc_url="https://github.com/accellera-official/systemc.git"

  echo "[fetch] systemc"
  echo "git clone --depth 1 --branch "${systemc_tag}" "${systemc_url}" "${src_dir}""
  git clone --depth 1 --branch "${systemc_tag}" "${systemc_url}" "${src_dir}"
}
configure_systemc() {
  _home_=$PWD
  src_dir="$1"
  build_dir="$2"
  install_dir="$3"
  version="${4}"

  echo "[configure] systemc"
  cmake -S "${src_dir}" -B "${build_dir}" \
    -D CMAKE_INSTALL_PREFIX="${install_dir}" \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_CXX_STANDARD=17
}
build_systemc() {
  src_dir="$1"
  build_dir="$2"
  install_dir="$3"
  version="${4}"

  echo "[build] systemc"
  cmake --build "${build_dir}" --parallel $(nproc)
}
install_systemc() {
  src_dir="$1"
  build_dir="$2"
  install_dir="$3"
  version="${4}"

  echo "[install] systemc"
  cmake --build "${build_dir}" --parallel $(nproc) --target install
}
cleanup_systemc() {
  src_dir="$1"
  build_dir="$2"
  install_dir="$3"
  version="${4}"

  echo "[clean-up] systemc"
  rm -rf "${src_dir}"
  rm -rf "${build_dir}"
}
setup_systemc() {
  src_dir="$1"
  build_dir="$2"
  install_dir="$3"
  version="${4}"

  if [ ! -f "${install_dir}/lib/libsystemc.so" ]; then
    fetch_systemc "$1" "$2" "$3" "${4}" && \
    configure_systemc "$1" "$2" "$3" "${4}" && \
    build_systemc "$1" "$2" "$3" "${4}" && \
    install_systemc "$1" "$2" "$3" "${4}" && \
    cleanup_systemc "$1" "$2" "$3" "${4}"
  fi
}


setup() {
  _what_=$1
  setup_${_what_} $2 $3 $4 $5 $6
}
