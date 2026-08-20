COMPOSE := docker compose --env-file .env -f docker/compose.yaml

.PHONY: init diagnose build shell check gui-auth sim real down

init:
	@test -f .env || cp .env.example .env

diagnose: init
	@echo "Projeto: $(CURDIR)"
	@echo "Dockerfile: $$(realpath docker/Dockerfile)"
	@echo "Imagem: $$($(COMPOSE) --profile dev config --images)"
	@grep -n -E 'UR_SIMULATION_GZ_COMMIT|ONROBOT_DESCRIPTION_COMMIT|apt-get update|existing_group|groupadd --gid' docker/Dockerfile

build: init
	$(COMPOSE) --profile dev build ur_cbf_dev

shell: init
	$(COMPOSE) --profile dev run --rm ur_cbf_dev

check: init
	$(COMPOSE) --profile dev run --rm ur_cbf_dev ./scripts/check_system.sh

gui-auth:
	@if [ -z "$${DISPLAY:-}" ]; then \
		echo "ERRO: DISPLAY nao esta definido no host."; \
		echo "Inicie uma sessao grafica X11/XWayland antes de executar make sim."; \
		exit 1; \
	fi
	@if ! command -v xhost >/dev/null 2>&1; then \
		echo "ERRO: comando xhost nao encontrado no host."; \
		echo "Instale-o com: sudo apt install x11-xserver-utils"; \
		exit 1; \
	fi
	@if ! xhost +SI:localuser:"$$(id -un)" >/dev/null; then \
		echo "ERRO: nao foi possivel autorizar o usuario local no servidor X."; \
		exit 1; \
	fi
	@echo "Acesso grafico X11 autorizado para o usuario $$(id -un)."

sim: init gui-auth
	$(COMPOSE) --profile sim up ur_cbf_sim

real: init
	$(COMPOSE) --profile real up ur_cbf_real

down:
	$(COMPOSE) --profile dev --profile sim --profile real down --remove-orphans
