# Running
```sh
ls xmake.lua Dockerfile planet-coastlinespbf-cleanedosmpbf.sec
docker build --progress=plain -t chriku-sqt .
docker run --rm -v $PWD/:/home/christian/sqt/ -p 8080:8080 chriku-sqt
```
ensure that the first `ls` command finds all files!