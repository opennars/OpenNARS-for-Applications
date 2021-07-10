./NAR pong 2000 > pong
./NAR pong2 2000 > pong2
./NAR alien 2000 > alien
./NAR cartpole 2000 > cartpole
echo bgcghbhaqqqqqqqqhbhadghaqqqqhbhassiiattqqajja.Q | ./NAR testchamber > testchamber

python3 ./misc/evaluation/visualize_demo_run.py pong
python3 ./misc/evaluation/visualize_demo_run.py pong2
python3 ./misc/evaluation/visualize_demo_run.py alien
python3 ./misc/evaluation/visualize_demo_run.py cartpole
python3 ./misc/evaluation/visualize_demo_run.py testchamber

ffmpeg -i pong.gif pong.mp4
ffmpeg -i pong2.gif pong2.mp4
ffmpeg -i alien.gif alien.mp4
ffmpeg -i cartpole.gif cartpole.mp4
ffmpeg -i testchamber.gif testchamber.mp4

ffmpeg -i pong.mp4 -filter:v fps=20 pong_20FPS.mp4
ffmpeg -i pong2.mp4 -filter:v fps=20 pong2_20FPS.mp4
ffmpeg -i alien.mp4 -filter:v fps=20 alien_20FPS.mp4
ffmpeg -i cartpole.mp4 -filter:v fps=20 cartpole_20FPS.mp4
ffmpeg -i testchamber.mp4 -filter:v fps=20 testchamber_20FPS.mp4
rm testchamber.mp4

sh CombineVideos.sh
