#!/bin/bash
# Quick API test script for GCS Swarm
BASE="http://localhost:5000"

echo "=== Testing GCS Swarm API ==="
echo ""

echo "[1] GET /api/swarm/status"
curl -s "$BASE/api/swarm/status" | python3 -m json.tool
echo ""

echo "[2] POST /api/swarm/connect (num_drones=3)"
curl -s -X POST "$BASE/api/swarm/connect" \
  -H "Content-Type: application/json" \
  -d '{"num_drones":3}' | python3 -m json.tool
echo ""

echo "[3] GET /api/swarm/status (after connect)"
curl -s "$BASE/api/swarm/status" | python3 -m json.tool
echo ""
