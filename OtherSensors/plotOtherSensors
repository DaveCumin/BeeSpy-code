FOLDER = "/Users/dcum007/Downloads/20250314_Arduino/"

##Get the files of interest
files = list.files(FOLDER)
files = files[which(grepl("_cap.txt",files))]

## set up the lables
labels = c()
data = list()

## get the data out
for(f in files){
  cat(f, "\n")
  ### do the business for each file
  if(file.size(file.path(FOLDER, f)) > 0){
    ## read in
    df = read.csv(file.path(FOLDER,f), header = F)
    
    ## change the names to be easier to read
    names(df) = c("time", "lab","value")
    
    ## update the labels
    labels = union(labels, unique(df$lab))
    
    ## get the timestamp from file
    ftime = strptime(strsplit(f, "_cap.txt")[[1]], format = "%Y_%m_%d_%H_%M_%S")
    ## make a timestamp
    df$timestamp = ftime + (df$time / 1000)
    
    ## now insert the timestamp and data into the data
    for(l in unique(df$lab)){
      idx = which(df$lab == l)
      tempData = data.frame(time= df$timestamp[idx], 
                    value = df$value[idx]
                    )
      data[[l]] = rbind(data[[l]], tempData)
    }
  }  
}


## plot the data
par(mfcol=c(4,1))
for(d in names(data)[1:4]){
  plot(data[[d]]$time, data[[d]]$value, 'p', 
       col=which(names(data)==d), lwd=3, 
       main=d, xlab = "Time", ylab="")  
}


# d= "eCO2"
# plot(data[[d]]$time, data[[d]]$value, 'p', 
#      col=which(names(data)==d), lwd=3, 
#      main=d, xlab = "Time", ylab="", ylim=c(0,10000))  
