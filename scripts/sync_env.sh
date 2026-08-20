#!/usr/bin/env bash
set -euo pipefail

target="${1:-.env}"
template="${2:-.env.example}"
shift "$(( $# >= 1 ? 1 : 0 ))"
shift "$(( $# >= 1 ? 1 : 0 ))"

if [[ ! -f "${template}" ]]; then
  echo "ERRO: modelo de ambiente nao encontrado: ${template}" >&2
  exit 1
fi

if [[ ! -f "${target}" ]]; then
  cp "${template}" "${target}"
fi

temporary="$(mktemp "${target}.tmp.XXXXXX")"
trap 'rm -f "${temporary}"' EXIT

template_value() {
  local key="$1"
  awk -F= -v key="${key}" '$1 == key {sub(/^[^=]*=/, ""); print; exit}' "${template}"
}

known_key() {
  local key="$1"
  grep -q -E "^${key}=" "${template}"
}

upsert() {
  local key="$1"
  local value="$2"
  awk -v key="${key}" -v value="${value}" '
    BEGIN { written = 0 }
    index($0, key "=") == 1 {
      if (!written) {
        print key "=" value
        written = 1
      }
      next
    }
    { print }
    END {
      if (!written) {
        print key "=" value
      }
    }
  ' "${target}" > "${temporary}"
  mv "${temporary}" "${target}"
  temporary="$(mktemp "${target}.tmp.XXXXXX")"
}

# Estas chaves identificam a revisao consolidada e sao gerenciadas pelo projeto.
for managed_key in IMAGE_TAG ONROBOT_TYPE; do
  managed_value="$(template_value "${managed_key}")"
  if [[ -z "${managed_value}" ]]; then
    echo "ERRO: ${managed_key} nao definido em ${template}." >&2
    exit 1
  fi
  upsert "${managed_key}" "${managed_value}"
done

# Novas opcoes ganham os valores padrao sem sobrescrever IPs e escolhas locais.
while IFS='=' read -r key value; do
  [[ "${key}" =~ ^[A-Z][A-Z0-9_]*$ ]] || continue
  if ! grep -q -E "^${key}=" "${target}"; then
    upsert "${key}" "${value}"
  fi
done < "${template}"

# A configuracao persistente pode ser feita pelo Makefile, sem editar .env.
for assignment in "$@"; do
  if [[ "${assignment}" != *=* ]]; then
    echo "ERRO: configuracao invalida: ${assignment}" >&2
    exit 1
  fi
  key="${assignment%%=*}"
  value="${assignment#*=}"
  if ! [[ "${key}" =~ ^[A-Z][A-Z0-9_]*$ ]] || ! known_key "${key}"; then
    echo "ERRO: chave de ambiente desconhecida: ${key}" >&2
    exit 1
  fi
  case "${key}" in
    IMAGE_TAG|ONROBOT_TYPE)
      echo "ERRO: ${key} e gerenciada por ${template}." >&2
      exit 1
      ;;
  esac
  upsert "${key}" "${value}"
done

image_tag="$(awk -F= '$1 == "IMAGE_TAG" {print $2; exit}' "${target}")"
onrobot_type="$(awk -F= '$1 == "ONROBOT_TYPE" {print $2; exit}' "${target}")"
echo "Ambiente sincronizado: IMAGE_TAG=${image_tag}; ONROBOT_TYPE=${onrobot_type}."
