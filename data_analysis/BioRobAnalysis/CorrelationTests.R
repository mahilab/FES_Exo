### Template for running correlation and regression in R
library(psych)
library(ggplot2)
library(readr)
library(ggpubr)

dof_types <- c("single","multi")
joints    <- c("elbow_","forearm_","wrist_fe_","wrist_ru_")
metrics   <- c("rms_torque","max_torque","rms_error")

for (dof_type in dof_types) {
  dataset <- read_csv(paste("C:\\Git\\FES_Exo\\data_analysis\\",dof_type,"_stats.csv",sep=""))
  
  dataset_adj <- dataset[-c(1,2,3,4),]
  
  for (joint in joints) {
    for (metric in metrics) {
      col_num <- which(colnames(dataset_adj) == paste(joint,metric,sep=""))
      local_data <- as.numeric(unlist(dataset_adj[col_num]))
      
      cor.out <- cor.test(dataset_adj$fes_share, local_data,
                          method = c("pearson"),
                          conf.level = 0.95)
      
      lm.out <- lm(local_data ~ dataset_adj$fes_share)
      
      print(paste(dof_type," ",joint,metric,": ",cor.out$p.value,", ",lm.out$coefficients[2],sep = ""))
    }
  }
}

# use to visualize specific models
ggscatter(dataset_adj, x = "fes_share", y = "forearm_rms_error", 
          add = "reg.line", conf.int = TRUE, 
          cor.coef = TRUE, cor.method = "pearson",
          xlab = "FES share", ylab = "Elbow RMS Torque")