FROM node:18-bullseye AS builder

WORKDIR /app

# Copy UI sources
COPY . /app

# Install dependencies and build --progress=false to reduce log size
RUN npm install --no-audit 

# Build-time environment variable (can be overridden during build)
ARG NEXT_PUBLIC_API_URL=http://localhost:5050/api
ENV NEXT_PUBLIC_API_URL=$NEXT_PUBLIC_API_URL

RUN npm run build

FROM node:18-bullseye-slim
WORKDIR /app
COPY --from=builder /app /app

# Runtime environment variable
ENV NODE_ENV=production
ENV NEXT_PUBLIC_API_URL=http://localhost:5050/api

EXPOSE 3000
CMD ["npm","run","start"]
