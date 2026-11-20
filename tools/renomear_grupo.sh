#!/bin/bash
# Renomeia o template do grupo 00 para o vosso grupo.
#   bash tools/renomear_grupo.sh g05
# Muda: pasta e pacote va_g00 -> va_g05, GRUPO no .env, nomes nos compose e no README.
set -e
novo="$1"
if [[ ! "$novo" =~ ^g[0-9]{2}$ ]]; then
    echo "uso: bash tools/renomear_grupo.sh gNN   (p. ex. g05)"; exit 1
fi
cd "$(dirname "$0")/.."
[ -d src/va_g00 ] || { echo "src/va_g00 não existe (já foi renomeado?)"; exit 1; }

git mv src/va_g00/va_g00 "src/va_g00/va_${novo}" 2>/dev/null || mv src/va_g00/va_g00 "src/va_g00/va_${novo}"
git mv src/va_g00/resource/va_g00 "src/va_g00/resource/va_${novo}" 2>/dev/null || mv src/va_g00/resource/va_g00 "src/va_g00/resource/va_${novo}"
git mv src/va_g00 "src/va_${novo}" 2>/dev/null || mv src/va_g00 "src/va_${novo}"

num="${novo#g}"
grep -rl --exclude-dir=.git --exclude-dir=data --exclude=renomear_grupo.sh \
    -e 'va_g00' -e 'g00' -e 'Grupo 00' -e 'grupo00' . \
  | xargs sed -i -e "s/va_g00/va_${novo}/g" -e "s/GRUPO=g00/GRUPO=${novo}/" \
                 -e "s/Grupo 00/Grupo ${num}/g" -e "s/grupo00/grupo${num}/g" \
                 -e "s/\${GRUPO:-g00}/\${GRUPO:-${novo}}/g"
echo "Feito. Confirmar com:  git status   e   docker compose build"
