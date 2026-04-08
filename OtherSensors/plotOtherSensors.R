FILE = "/Users/dcum007/Downloads/JAMHive_NONFaradayed_other.csv"
d = read.csv(FILE, header = F)
head(d)
names(d) = c("time", "CO2", "Hum","Temp")
d$time = as.POSIXct(d$time)
par(mfcol=c(3,1))
plot(d$time, d$CO2, ylim = c(0,10000))
plot(d$time, d$Hum)
plot(d$time, d$Temp, ylim=c(30.5, 31.5))


library(ggplot2)
library(dplyr)
library(lubridate)
library(patchwork)

# --- Generate night rectangles (same as before) ---
date_seq <- seq(as.Date(min(d$time)), as.Date(max(d$time)) + 1, by = "day")

night_rects <- tibble(
  xmin = as.POSIXct(paste(date_seq[-length(date_seq)], "18:00:00")),
  xmax = as.POSIXct(paste(date_seq[-1],                "06:00:00"))
) %>%
  mutate(
    xmin = pmax(xmin, min(d$time)),
    xmax = pmin(xmax, max(d$time))
  ) %>%
  filter(xmin < xmax)

# --- Shared night shading layer ---
night_layer <- geom_rect(
  data = night_rects,
  aes(xmin = xmin, xmax = xmax, ymin = -Inf, ymax = Inf),
  inherit.aes = FALSE,
  fill = "grey80", alpha = 0.5
)

# --- Individual plots ---
p_co2 <- ggplot(d, aes(x = time, y = CO2)) +
  night_layer +
  geom_line() +
  scale_y_continuous(limits = c(0, 10000)) +
  labs(x = NULL, y = "CO₂ (ppm)") +
  theme_minimal()

p_hum <- ggplot(d, aes(x = time, y = Hum)) +
  night_layer +
  geom_line(colour = "steelblue") +
  scale_y_continuous(limits = c(50, 90)) +
  labs(x = NULL, y = "Humidity (%)") +
  theme_minimal()

p_temp <- ggplot(d, aes(x = time, y = Temp)) +
  night_layer +
  geom_line(colour = "tomato") +
  scale_y_continuous(limits = c(30, 32)) +
  labs(x = "Time", y = "Temperature (°C)") +
  theme_minimal()

# --- Stack with patchwork ---
p_co2 / p_hum / p_temp


# ###
# FOLDER = "/Users/dcum007/Downloads/20250314_Arduino/"
# 
# ##Get the files of interest
# files = list.files(FOLDER)
# files = files[which(grepl("_cap.txt",files))]
# 
# ## set up the lables
# labels = c()
# data = list()
# 
# ## get the data out
# for(f in files){
#   cat(f, "\n")
#   ### do the business for each file
#   if(file.size(file.path(FOLDER, f)) > 0){
#     ## read in
#     df = read.csv(file.path(FOLDER,f), header = F)
#     
#     ## change the names to be easier to read
#     names(df) = c("time", "lab","value")
#     
#     ## update the labels
#     labels = union(labels, unique(df$lab))
#     
#     ## get the timestamp from file
#     ftime = strptime(strsplit(f, "_cap.txt")[[1]], format = "%Y_%m_%d_%H_%M_%S")
#     ## make a timestamp
#     df$timestamp = ftime + (df$time / 1000)
#     
#     ## now insert the timestamp and data into the data
#     for(l in unique(df$lab)){
#       idx = which(df$lab == l)
#       tempData = data.frame(time= df$timestamp[idx], 
#                     value = df$value[idx]
#                     )
#       data[[l]] = rbind(data[[l]], tempData)
#     }
#   }  
# }
# 
# 
# ## plot the data
# par(mfcol=c(4,1))
# for(d in names(data)[1:4]){
#   plot(data[[d]]$time, data[[d]]$value, 'p', 
#        col=which(names(data)==d), lwd=3, 
#        main=d, xlab = "Time", ylab="")  
# }
# 
# 
# # d= "eCO2"
# # plot(data[[d]]$time, data[[d]]$value, 'p', 
# #      col=which(names(data)==d), lwd=3, 
# #      main=d, xlab = "Time", ylab="", ylim=c(0,10000))  
