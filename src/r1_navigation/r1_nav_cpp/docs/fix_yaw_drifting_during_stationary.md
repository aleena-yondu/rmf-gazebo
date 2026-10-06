```bash
ros2 topic echo /ouster/imu --field angular_velocity.z --no-arr 2>/dev/null | tee /tmp/gyro_z.txt
```

```python
python3 -c "
data = [float(l) for l in open('/tmp/gyro_z.txt') if
l.strip().replace('-','').replace('.','').isdigit()]
print(f'Mean bias: {sum(data)/len(data):.6f} rad/s')
print(f'Samples: {len(data)}')
"
```

```

Expected from raw bias: 0.008438 rad/s × 299s = 2.53°
Actual measured drift:  1.93°

Mean bias: 0.000232 rad/s
Samples: 4446

```