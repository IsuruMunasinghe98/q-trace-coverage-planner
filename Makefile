.PHONY: install test example evaluate optimize

install:
	python -m pip install -e .

test:
	python -m unittest discover -s tests -v

example:
	python -m qtrace plan --dataset data/evaluation_set.txt --map-id 1 --config configs/global.toml --output results/example

evaluate:
	python -m qtrace evaluate --dataset data/evaluation_set.txt --config configs/global.toml --output results/evaluation

optimize:
	python -m qtrace optimize --dataset data/optimization_set.txt --scope global --trials 100 --seed 42 --output results/optimization.json
