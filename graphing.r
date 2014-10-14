require(ggplot2)
require(grid)

setwd("~/gitRepos/fall2014/poe/labThree/poe_motorcontroller")
sweep_df = read.csv(file = 'sweep.csv', header = FALSE)
colnames(sweep_df) = c('set_pos', 'meas_pos', 'error', 'signal')
sweep_time = data.frame()
sweep_time[1:nrow(sweep_df),"time"] = seq(0, 0.002*(nrow(sweep_df)-1), by = 0.002)
ploting_df = data.frame()
for(j in 1:ncol(sweep_df)){
  str_val = colnames(sweep_df)[j]
  init_ind = (j-1)*i
  for(i in 1:nrow(sweep_df)){
    ploting_df[(i+init_ind),'value'] = sweep_df[i,j]
    ploting_df[(i+init_ind),'time'] = sweep_time[i,'time']
    ploting_df[(i+init_ind),'type'] = colnames(sweep_df)[j]
  }
}
ggplot(data = ploting_df, aes(x = time, y = value, colour = factor(type)))+
  geom_line(size = rel(1))+
  labs(x = "Time (s)", y = "Degrees or Arduino Out", title = "Sinusoidal Following of DC Motor")+
  theme(legend.text=element_text(size=rel(1.25)),axis.title.x = element_text(size=rel(1)), axis.title.y = element_text(size=rel(1)), title = element_text(size=rel(1.75)), axis.text.x = element_text(size=rel(1.5)), axis.text.y = element_text(size=rel(1.75)))+
  scale_color_manual(name = 'Signal Type', values = c("Red", "Blue", "Black", "DarkGreen"), labels = c('Error', 'Measured Position', 'Set Position', 'Signal to Motor'))

step_df = read.csv(file = 'step90.csv', header = FALSE)
step_df = step_df[1:1000,]
colnames(step_df) = c('set_pos', 'meas_pos', 'error', 'signal')
step_time = data.frame()
step_time[1:nrow(step_df),"time"] = seq(0, 0.002*(nrow(step_df)-1), by = 0.002)
ploting_df = data.frame()
for(j in 1:ncol(step_df)){
  str_val = colnames(step_df)[j]
  init_ind = (j-1)*i
  for(i in 1:nrow(step_df)){
    ploting_df[(i+init_ind),'value'] = step_df[i,j]
    ploting_df[(i+init_ind),'time'] = step_time[i,'time']
    ploting_df[(i+init_ind),'type'] = colnames(step_df)[j]
  }
}
ggplot(data = ploting_df, aes(x = time, y = value, colour = factor(type)))+
  geom_line(size = rel(1))+
  labs(x = "Time (s)", y = "Degrees or Arduino Out", title = "Step Responce of DC Motor")+
  theme(legend.text=element_text(size=rel(1.25)),axis.title.x = element_text(size=rel(1)), axis.title.y = element_text(size=rel(1)), title = element_text(size=rel(1.75)), axis.text.x = element_text(size=rel(1.5)), axis.text.y = element_text(size=rel(1.75)))+
  scale_color_manual(name = 'Signal Type', values = c("Red", "Blue", "Black", "DarkGreen"), labels = c('Error', 'Measured Position', 'Set Position', 'Signal to Motor'))

encoder = read.csv(file = 'encoder.csv', header = FALSE)
plot(x = c(1:nrow(encoder)), y = encoder[1:nrow(encoder),], xlab = 'Datapoint', ylab = 'IR Sensor Responce', main = 'Encoder Calibration Plot')
lines(x = c(0, 3450), y = c(70, 70), col = 'blue', lwd = 5)