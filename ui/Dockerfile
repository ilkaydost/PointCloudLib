FROM node:18-bullseye AS builder

WORKDIR /app

# Copy UI sources
COPY . /app

# Install dependencies and build
RUN npm install --prefer-offline --no-audit --progress=false
RUN npm run build

FROM node:18-bullseye-slim
WORKDIR /app
COPY --from=builder /app /app

ENV NODE_ENV=production
EXPOSE 3000
CMD ["npm","run","start"]
