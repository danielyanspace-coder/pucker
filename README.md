# Pucker

SaaS для расчёта трёхмерной укладки коробок на палеты и в кузова транспорта.

- `docs/TZ.md` — ТЗ заказчика на алгоритм.
- `docs/DECISIONS.md` — принятые решения и план.
- `docs/demo/primer-eur.html` — 3D-результат «Примера» заказчика (открыть в браузере).
- `benchmarks/data/example.xlsx` — пример входных данных.
- `presets/places.json` — каталог мест погрузки (палеты, Газель, фуры, контейнеры).
- `engine/core` — ядро упаковки на Rust, `engine/cli` — утилита командной строки `pucker`.

## Запуск ядра

Нужен Rust (stable).

```sh
cargo build --release

# Упаковать Excel на EUR-палеты (до 3 шт.), 30 секунд на поиск, 3D-отчёт в HTML
./target/release/pucker pack --items benchmarks/data/example.xlsx --place PALLET_EUR:3 \
    --time-limit 30 --out result.json --html result.html

# Автоподбор между Газелью и 20-футовым контейнером
./target/release/pucker pack --items benchmarks/data/example.xlsx --place GAZELLE_STANDARD \
    --place CONTAINER_20FT --auto

# Своё место: ширина×глубина×высота (мм), грузоподъёмность (кг), количество
./target/release/pucker pack --items items.xlsx --custom 2100x4200x1900:1500:1

# Эталонные задачи (ТЗ §39) и тесты
./target/release/pucker bench --time-limit 10 --example benchmarks/data/example.xlsx
cargo test --release

# Перепроверить готовый результат независимым валидатором
./target/release/pucker pack --items benchmarks/data/example.xlsx --request-out req.json --out res.json
./target/release/pucker validate --request req.json --result res.json
```

Excel: колонки ищутся по названиям (Артикул, Наименование, Масса/Вес, Длина, Ширина,
Высота, Хрупкость, Количество, необязательно — Допустимая нагрузка, Приоритет).
Единицы длины берутся из заголовка («см», «мм»), по умолчанию сантиметры. Хрупкость — по ТЗ:
1 = самый хрупкий; для обратной шкалы `--fragility-scale inverted --fragility-levels 8`;
«да/нет» тоже понимается.

Профилирование попыток поиска: `PUCKER_TRACE=1 pucker pack …`.
