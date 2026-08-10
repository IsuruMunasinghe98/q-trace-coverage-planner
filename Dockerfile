FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    MPLBACKEND=Agg

WORKDIR /workspace
COPY pyproject.toml README.md ./
COPY src ./src
COPY configs ./configs
COPY data ./data
RUN python -m pip install --no-cache-dir .

CMD ["qtrace", "plan", "--dataset", "data/evaluation_set.txt", "--map-id", "1", "--config", "configs/global.toml", "--output", "results/example"]
