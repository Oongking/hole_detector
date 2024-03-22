import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt
import pywt

import torch
import torch.nn as nn
import torch.nn.functional as F

from torch.utils.data import Dataset
from torch.utils.data import DataLoader
import copy

from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay


device = (
    "cuda"
    if torch.cuda.is_available()
    else "mps"
    if torch.backends.mps.is_available()
    else "cpu"
)
print(f"Using {device} device")

Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.003,(0,0,0))

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a = (vec1 / np.linalg.norm(vec1)).reshape(3)
    b = (vec2 / np.linalg.norm(vec2)).reshape(3)
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)

    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    angle = np.arccos(c)
    return rotation_matrix,angle


def circle_fiting(points):
    

    x = points[:,0].T 
    y = points[:,1].T
    xy = points[:,:2]

    # print(f"x : {x.shape}")
    # print(f"y : {y.shape}")
    # print(f"xy : {xy.shape}")
    # print(f"xy : {np.power(np.linalg.norm(xy,axis=1),2)}")
    B = np.power(np.linalg.norm(xy,axis=1),2).T
    A = np.array([x*2, y*2,np.ones(x.shape[0])]).T
    # print(f"A : {A}")
    # print(f"B : {B}")
    # print(f"np.linalg.lstsq(A,B) : {np.linalg.lstsq(A,B, rcond=None)[0]}")
    ans = np.linalg.lstsq(A,B, rcond=None)[0][:3]
    r = np.sqrt(ans[2]+np.power(ans[0],2)+np.power(ans[1],2))
    # print(f"radius : {r}")
    # print(f"ans : {ans}")
    # print(f"ans : {ans[2]+np.power(ans[0],2)+np.power(ans[1],2)}")

    return ans[:2],r

def load_data(path,name,num,class_num):

    datasets = []

    for i in range(1,num+1):

        pcd_original = o3d.io.read_point_cloud(f"{path}/{name}{i}.pcd")
        original_point = np.asarray(pcd_original.points)
        nan_rows = np.isnan(original_point).any(axis=1)

        # Remove rows with NaN values
        original_point_without_nan = original_point[~nan_rows]

        pcd_original.points = o3d.utility.Vector3dVector(original_point_without_nan)

        plane_model, inliers = pcd_original.segment_plane(distance_threshold=0.1,
                                         ransac_n=3,
                                         num_iterations=1000)

        # pcdcen = pcdcen.select_by_index(inliers)
        # xyz = np.asarray(pcdcen.points)

        # find plane and project point to plane
        [a, b, c, d] = plane_model
        n = np.array([a, b, c])/np.linalg.norm([a,b,c])

        # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        # d : distance to plan  n : Normal vector of plane
        # d = (a*xyz[:,0]+b*xyz[:,1]+c*xyz[:,2]+d)/np.linalg.norm([a,b,c])
        
        tfm_plane = np.eye(4)
        tfm_plane[:3,:3] = rotation_matrix_from_vectors([0,0,1],n)[0]
        tfm_plane[:3,3] = pcd_original.points[0]
        inv_tfm_plane = np.linalg.inv(tfm_plane)
        pcd = copy.deepcopy(pcd_original)
        pcd.transform(inv_tfm_plane)

        points = np.asarray(pcd.points)
        if np.isnan(points).any():
            print(f"i : {i}")
            print(f"++++++++++++++NAN++++++++++++++")
            # o3d.visualization.draw_geometries([pcd_original])
            # o3d.visualization.draw_geometries([pcd])

            # print(f"points : {points}")

        [x,y],r = circle_fiting(points)

        tfm_center = np.eye(4)
        tfm_center[:2,3] = [x,y]
        
        inv_tfm_center = np.linalg.inv(tfm_center)
        pcd.transform(inv_tfm_center)
        points = np.asarray(pcd.points)[:,:2]*1000
    

        datasets.append([points,r*1000])

        
    # remove 
    list_long = len(datasets)-(len(datasets)%100)
    datasets = datasets[:list_long]

    # o3d.visualization.draw_geometries([pcd,Realcoor])
    print(f"len : {len(datasets)}")
    # print(f"ex : {datasets[0]}")

    class_tag = np.full([len(datasets)],[class_num])

    return datasets,class_tag




class Classification_Dataset(Dataset):
    def __init__(self, data_features,data_labels):

        self.data_features = data_features
        self.data_labels = np.array(data_labels).reshape(-1,1)
        
    def __len__(self):
        return len(self.data_labels)

    def __getitem__(self, idx):

        point = self.data_features[idx][0]
        radius = self.data_features[idx][1]

        # print(f"point[:5] : {point[:5]}")
        # print(f"radius : {radius}")

        point = np.array(point)
        radius = np.array(radius)


        data = np.c_[point, np.full(point.shape[0],radius)]

        num = 150-point.shape[0]
        data = np.vstack((data,np.zeros((num,3))))
        # print(f"data.shape : {data.shape}")
        # print(f"data[:5] : {data[:5]}")

        train = torch.from_numpy(data).to(device='cuda')
        label = torch.from_numpy(self.data_labels[idx]).to(device='cuda')

        return train, label.squeeze()


def test_loop(dataloader, model, loss_fn, list_tag = False):
    # Set the model to evaluation mode - important for batch normalization and dropout layers
    # Unnecessary in this situation but added for best practices
    model.eval()
    size = len(dataloader.dataset)
    num_batches = len(dataloader)
    test_loss, correct = 0, 0

    predicts = []
    targets = []

    # Evaluating the model with torch.no_grad() ensures that no gradients are computed during test mode
    # also serves to reduce unnecessary gradient computations and memory usage for tensors with requires_grad=True
    with torch.no_grad():
        for X, y in dataloader:

            # print(X.size())
            pred = model(X.float())

            # print(f"pred : {pred}")
            # print(f"y : {y}")
            y = y.type(torch.LongTensor).to(device='cuda')
            test_loss += loss_fn(pred, y).item()

            correct += (pred.argmax(1) == y).type(torch.float).sum().item()
            predicts.append(y.cpu().numpy())
            targets.append(pred.data.max(1, keepdim=True)[1].cpu().numpy().reshape(-1))


    test_loss /= num_batches
    correct /= size
    print(f"Test Error: \n Accuracy: {(100*correct):>0.1f}%, Avg loss: {test_loss:>8f} \n")

    if list_tag:
        return 100*correct,test_loss,predicts,targets
    else:
        return 100*correct,test_loss



class NeuralNetwork(nn.Module):
    def __init__(self):
        super().__init__()

        self.flatten = nn.Flatten()
        
        self.CNN = nn.Sequential(
        nn.Conv2d(1, 6, kernel_size=(3, 3), stride=(1, 1), padding=(1, 1)),
        nn.LeakyReLU(inplace=True),
        nn.BatchNorm2d(6),
        nn.AvgPool2d(kernel_size=2, stride=2, padding=1,  ceil_mode=False),
        )
        
        self.conv1d = nn.Conv1d(150, 150, 3)

        self.full_connected = nn.Sequential(
            nn.Linear(456, 912),
            nn.LeakyReLU(),
            nn.Linear(912, 912),
            nn.LeakyReLU(),
            nn.Linear(912, 2),
        )

    def forward(self, x):
        # print("x shape in:", x.shape)
        
        x = self.conv1d(x)
        # print("x shape conv1d:", x.shape)

        x = x.view(x.shape[0], 1, x.shape[1],x.shape[2])
        # print("x shape view:", x.shape)

        x = self.CNN(x)
        # print("x shape cnn:", x.shape)

        x = self.flatten(x)
        # print("x shape flat:", x.shape)

        logits = self.full_connected(x)


        return logits
    
model = NeuralNetwork().to(device)
print(model)


if __name__=="__main__":

    package_path = "/home/oongking/AI/final_project/data"

    acceptable_hole_feature,acceptable_hole_tag = load_data(f"{package_path}/acceptable_hole/test_in",
                                            "accept",5000,1)

    unacceptable_hole_feature,unacceptable_hole_tag = load_data(f"{package_path}/unacceptable_hole/test_in",
                                            "unaccept",5000,0)


    data_features = acceptable_hole_feature + unacceptable_hole_feature
    data_labels = np.concatenate((acceptable_hole_tag,unacceptable_hole_tag))



    print(f"data_features : {len(data_features)}")
    print(f"data_labels : {len(data_labels)}")

    label_dict = {
        "accept": 0, 
        "unaccept": 1, 
    }
    class_names = list(label_dict.keys())

    hole_dataset = Classification_Dataset(data_features,data_labels)
    accepthole_dataset = Classification_Dataset(acceptable_hole_feature,acceptable_hole_tag)
    unaccepthole_dataset = Classification_Dataset(unacceptable_hole_feature,unacceptable_hole_tag)

    num = hole_dataset.__len__()
    train, label = hole_dataset.__getitem__(0)
    # print(f"train : {train}")
    # print(f"label : {label}")

    train_set, test_set = torch.utils.data.random_split(hole_dataset, [8000,2000])

    # batch size 1 per time because point not thae same
    train_dataloader = DataLoader(train_set, batch_size=8, shuffle=True)
    test_dataloader = DataLoader(test_set, batch_size= 8,shuffle=True)
    hole_datasetloader = DataLoader(hole_dataset, batch_size= 8,shuffle=True)
    accepthole_datasetloader = DataLoader(accepthole_dataset, batch_size= 8,shuffle=True)
    unaccepthole_datasetloader = DataLoader(unaccepthole_dataset, batch_size= 8,shuffle=True)


    train_features, train_labels = next(iter(train_dataloader))

    print(f"Feature batch shape: {train_features.size()}")
    print(f"Labels batch shape: {train_labels.size()}")


    learning_rate_fast = 5e-3
    learning_rate_mid = 1e-3
    learning_rate_low = 5e-4
    learning_rate_lowV2 = 1e-4


    loss_fn = nn.CrossEntropyLoss()
    optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate_fast)

    epochs = 1000
    max_accuracy = 0
    optimizer_stage = 0

    patient = 200

    no_update = 0
    name = "CNN_full"
    save_path = "/home/oongking/AI/final_project/script/CNN"

    model = torch.load(f"{save_path}/CNN_full_1000_best.pth").to(device='cuda')

    test_accuracy,test_loss,predicts,targets = test_loop(hole_datasetloader, model, loss_fn,True)

    predicts = np.concatenate( predicts, axis=0 )
    targets = np.concatenate( targets, axis=0 )

    # Build confusion matrix
    cf_matrix = confusion_matrix(targets, predicts)

    plt.figure(figsize = (12,7))
    ConfusionMatrixDisplay(cf_matrix,display_labels = class_names).plot()
    plt.savefig(f'{save_path}/confusion_matrix.png')

    print(f"IMU_dataloader test_accuracy : {test_accuracy}, test_loss : {test_loss}")

    test_accuracy,test_loss = test_loop(accepthole_datasetloader, model, loss_fn)
    print(f"accepthole_datasetloader test_accuracy : {test_accuracy}, test_loss : {test_loss}")

    test_accuracy,test_loss = test_loop(unaccepthole_datasetloader, model, loss_fn)
    print(f"unaccepthole_datasetloader test_accuracy : {test_accuracy}, test_loss : {test_loss}")
