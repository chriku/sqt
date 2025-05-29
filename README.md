# Running

```sh
ls xmake.lua Dockerfile planet-coastlinespbf-cleanedosmpbf.sec
docker build --progress=plain -t chriku-sqt .
docker run --rm -t -v "$PWD"/:/home/christian/sqt/ -p 8080:8080 chriku-sqt
```
ensure that the first `ls` command finds all files!

for e.g. docker windows, replace all incompatible parts (e.g. `$PWD`) with correct alternatives
