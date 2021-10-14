#!/bin/bash
ssh -p 22 admin@$1 date -u --set=\"$(date)\"

