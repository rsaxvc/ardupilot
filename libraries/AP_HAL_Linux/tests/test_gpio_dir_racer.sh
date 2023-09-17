UNRELATED_PIN=13
while :
do
    sudo raspi-gpio set ${UNRELATED_PIN} ip
    sudo raspi-gpio set ${UNRELATED_PIN} op
done
