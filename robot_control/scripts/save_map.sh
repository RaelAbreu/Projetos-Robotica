set -euo pipefail
if [ $# -ne 1 ]; then echo "uso: $0 <caminho/nome_sem_extensao>"; exit 1; fi
NAME="$(readlink -m "${1/#\~/$HOME}")"
mkdir -p "$(dirname "$NAME")"
echo ">> salvando mapa de ocupação em ${NAME}.{pgm,yaml}"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '${NAME}'}}"
echo ">> serializando grafo de poses em ${NAME}.{posegraph,data}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${NAME}'}"
