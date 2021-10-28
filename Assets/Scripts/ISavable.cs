namespace Dora {
    public interface ISavable<T> {
        public T SaveState();

        public void RestoreState(T stateInfo);
    }
}