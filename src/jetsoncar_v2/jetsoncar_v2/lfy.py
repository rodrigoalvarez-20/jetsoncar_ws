import torch
import torch.nn as nn


class RCCarHead(nn.Module):
    """
    
    """
    def __init__(self, in_features = 128, hidden_dim = 512, dropout = 0.1, device = "cuda"):
        super(RCCarHead, self).__init__()
        
        # Simple FC layers for regression
        self.in_features = in_features
        self.hidden_dim = hidden_dim
        self.dropout = dropout
        self.device = device
        self.fc1 = nn.Linear(in_features, hidden_dim * 8, device=device)
        self.fc2 = nn.Linear(hidden_dim * 8, hidden_dim * 8, device=device)
        self.fc3 = nn.Linear(hidden_dim * 8, hidden_dim * 6, device=device)
        self.fc4 = nn.Linear(hidden_dim * 6, hidden_dim * 4, device=device)
        self.fc5 = nn.Linear(hidden_dim * 4, hidden_dim * 2, device=device)
        self.fc6 = nn.Linear(hidden_dim * 2, hidden_dim, device=device)
        self.fc7 = nn.Linear(hidden_dim, hidden_dim // 2, device=device)
        self.relu = nn.ReLU()
        self.dropout = nn.Dropout(dropout)
        # Output layer: 2 values (Steering, Throttle)
        # Steering: [-1.0, 1.0], Throttle: [0.0, 1.0]
        self.fc_output = nn.Linear(hidden_dim // 2, 2, device=device)
    
    def save_checkpoint(self, path):
        print(f'Saving checkpoint {path}')
        torch.save({
            'in_features': self.in_features,
            'hidden_dim': self.hidden_dim,
            #'dropout': self.dropout,
            'model_state_dict': self.state_dict()
        }, path)

    @staticmethod
    def load_checkpoint(path, device = "cuda") -> 'RCCarHead':
        checkpoint = torch.load(path, weights_only=False, map_location=device)
        model = RCCarHead(
            in_features=checkpoint["in_features"],
            hidden_dim=checkpoint["hidden_dim"],
            #dropout=checkpoint['dropout'],
            device=device
        )
        model.load_state_dict(checkpoint['model_state_dict'])
        return model.to(device)

    def forward(self, x: torch.tensor):
        # x is the output from the shared feature extractor (YOLOS backbone)
        # Global Average Pooling to reduce feature map to a vector
        x = x.to(self.device)
        
        if x.dim() == 4:
            x = x.mean(dim=[-2, -1]) # Global average pool over HxW
        
        x = self.dropout(self.relu(self.fc1(x)))
        x = self.dropout(self.relu(self.fc2(x)))
        x = self.dropout(self.relu(self.fc3(x)))
        x = self.dropout(self.relu(self.fc4(x)))
        x = self.dropout(self.relu(self.fc5(x)))
        x = self.dropout(self.relu(self.fc6(x)))
        x = self.dropout(self.relu(self.fc7(x)))
        # Apply Tanh to steering (output[0]) for [-1, 1] range 
        # Apply Sigmoid to throttle (output[1]) for [0, 1] range
        output = self.fc_output(x)
        #print(output.shape)
        steering = torch.tanh(output[:, 0].unsqueeze(1))
        throttle = torch.sigmoid(output[:, 1].unsqueeze(1))
        
        #print(steering[:,0,:].shape, throttle[:,0,:].shape)

        return torch.cat([steering, throttle], dim=1) # Shape (Batch_size, 2)

#class LeopoldoFollowerModel:
#
#    def __init__(
#        self,
#        yolos_model="hustle/yolos-base",
#        car_model=None,
#        id2label={0: "No object"},
#        hf_cache="~/.cache/hugginface",
#    ):
#
#        self.image_processor = YolosImageProcessor.from_pretrained(
#            yolos_model, cache_dir=hf_cache
#        )
#        self.obj_model = YolosForObjectDetection.from_pretrained(
#            yolos_model,
#            num_labels=len(id2label),
#            ignore_mismatched_sizes=True,
#            cache_dir=hf_cache,
#        )
#
#        if car_model:
#            # Cargar los pesos del modelo
#            self.rc_car_model = 
#        else:
#            # Inicializar la capa/cabeza adicional
#            self.rc_car_model = 
#
