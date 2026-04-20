# docker build --target nonroot-dev -t navcat-dev .
# docker run -it --rm -p 5173:5173 --name navcat-dev-container -v ${PWD}:/home/nonroot/app navcat-dev /bin/bash

# pnpm i
# pnpm build

# cd examples/
# pnpm i
# pnpm dev --host 0.0.0.0

# ====================================================================== base ======================================================================
FROM node:22.22.2-bookworm-slim AS root

ENV FORCE_COLOR=1

RUN echo 'root:root' | chpasswd

RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install -y curl net-tools

# https://github.com/npm/cli/issues/9151
RUN npm i --location=global npm@~11.10.0
RUN npm i --location=global npm@latest
RUN npm i --location=global pnpm npm-check-updates serve

# --------------------------------------------------------------------------------------------------------------------------------------------------

FROM root AS nonroot

RUN userdel -r node
RUN groupadd --gid 1000 nonroot && useradd --uid 1000 --gid nonroot --shell /bin/bash --create-home nonroot

USER nonroot
WORKDIR /home/nonroot/app

# ====================================================================== dev =======================================================================

FROM nonroot AS nonroot-dev

EXPOSE 5173
