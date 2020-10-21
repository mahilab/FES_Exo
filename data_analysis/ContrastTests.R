### Planned contrasts in place of a one-way ANOVA

library(car)
library(emmeans)
library(multcomp)
library(ggplot2)

### Import Dataset the csv file "contrast_data.csv"

dof_types <- c("single","multi")
joints    <- c("elbow_","forearm_","wrist_fe_","wrist_ru_")
metrics   <- c("rms_torque","max_torque","rms_error")

share_vals <- seq(0.0,0.9,0.1)
variable_names <- c("share0.0","share0.1","share0.2","share0.3","share0.4","share0.5","share0.6","share0.7","share0.8","share0.9")

for (dof_type in dof_types) {
  dataset <- read_csv(paste("C:\\Git\\FES_Exo\\data_analysis\\",dof_type,"_stats.csv",sep=""))
  if (dof_type == "single") {
    dataset[40,] = c(0.4,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN)
  }
  
  for (joint in joints) {
    for (metric in metrics) {
      
      # #testing
      # dof_type <- "multi"
      # joint <- "wrist_fe_"
      # metric <- "rms_error"
      
      contrast_frame <- data.frame(matrix(ncol = 0, nrow = 4))
      for (share_val in share_vals) {
        data_slice <- dataset[(abs(dataset$fes_share-share_val) < 0.001),]  
        
        col_num <- which(colnames(data_slice) == paste(joint,metric,sep=""))
        
        contrast_frame[paste("share",share_val,sep="")] <- as.numeric(unlist(data_slice[col_num]))
      }
      
      ### Convert from wide to long form for contrasts
      contrast_long <- stack(contrast_frame)
      colnames(contrast_long) <- c("response","condition")
      
      ### reverse the columns so the condition is in the first column and the response is in the second column
      contrast_long <- contrast_long[,c(2,1)]
      
      ggplot(data=contrast_long, aes(x=condition,y=response)) +
        geom_boxplot()
      
      res.lm <- lm(response ~ condition, data=contrast_long)
      
      ### Check to make sure you know the order of the conditions/means in data
      levels(contrast_long$condition)
      
      leastsquares <- lsmeans(res.lm,"condition")
      
      Contrasts <- list(cntrl_vs_other = c(9,-1,-1,-1,-1,-1,-1,-1,-1,-1))
      
      contrast_result <- summary(contrast(leastsquares,Contrasts))   # unadjusted p-values
      
      print(paste(dof_type," ",joint,metric,": ",contrast_result$p.value,sep = ""))
    }
  }
}
