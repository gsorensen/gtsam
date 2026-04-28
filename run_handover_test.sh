for r in none gm tukey; do
    _build/programs/ifac_wc_2026 \
        --preint se3 --bias cb \
        --handover angle-range --robust $r \
        --compass-lag-ticks 0 \
        --noise-scaling 10 \
        --bias-scaling 50 \
        --baro-origin-msl 100
done
