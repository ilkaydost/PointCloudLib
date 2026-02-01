FROM node:18-bullseye AS builder

WORKDIR /app

# Copy UI sources
COPY . /app

# Install dependencies and build --progress=false to reduce log size
RUN npm install --no-audit 
RUN npm run build

FROM node:18-bullseye-slim
WORKDIR /app
COPY --from=builder /app /app

ENV NODE_ENV=production
EXPOSE 3000
CMD ["npm","run","start"]
