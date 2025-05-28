FROM debian:trixie
ENV XMAKE_ROOT y
RUN apt-get update && apt-get install -y lsb-release wget gnupg
RUN wget https://xmake.io/shget.text -O - | bash
RUN wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc && echo "deb http://apt.llvm.org/unstable/ llvm-toolchain main" >> /etc/apt/sources.list && echo "deb-src http://apt.llvm.org/unstable/ llvm-toolchain main" >> /etc/apt/sources.list
RUN echo "deb https://deb.debian.org/debian experimental main" >> /etc/apt/sources.list
RUN apt-get update && apt-get install -y -t experimental libstdc++-15-dev clang lld build-essential libllvm-ocaml-dev libllvm19 llvm llvm-dev llvm-runtime glslang-dev glslang-tools glslc
RUN apt-get update && apt-get install -y libabsl-dev libprotobuf-dev protobuf-compiler
WORKDIR /app
COPY xmake* .
RUN /root/.local/bin/xmake f -m release -c -y
COPY *.proto .
COPY src src
RUN /root/.local/bin/xmake -v -y
CMD /app/build/linux/x86_64/release/sqt
EXPOSE 8080